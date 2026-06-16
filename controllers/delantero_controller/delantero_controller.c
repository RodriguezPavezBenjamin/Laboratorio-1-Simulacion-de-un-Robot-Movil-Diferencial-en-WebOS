#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define CRUISE_SPEED (0.65 * MAX_SPEED)
#define TURN_SPEED (0.42 * MAX_SPEED)
#define SENSOR_THRESHOLD 80.0
#define WAYPOINT_TOLERANCE 0.12
#define GRID_ROWS 7
#define GRID_COLS 10
#define CELL_SIZE 0.9
#define GRID_ORIGIN_X -4.5
#define GRID_ORIGIN_Y -2.7
#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define LOG_INTERVAL_STEPS 32
#define MAX_ROUTE_POINTS (GRID_ROWS * GRID_COLS)
#define LARGE_COST 1000000

typedef struct {
  int fila;
  int col;
} Punto;

typedef struct {
  double x;
  double y;
  double theta;
} Pose2D;

typedef struct {
  double values[8];
  double left;
  double right;
  double front;
} ProximityState;

typedef struct {
  double left;
  double right;
  double prev_left;
  double prev_right;
} EncoderState;

typedef struct {
  int parent_index;
  int g_cost;
  int f_cost;
  bool open;
  bool closed;
} AStarNode;

typedef struct {
  const char *name;
  Punto start;
  Punto goal;
  int grid[GRID_ROWS][GRID_COLS];
} ScenarioConfig;

typedef struct {
  int avoidance_events;
  int front_avoidance_events;
  int left_avoidance_events;
  int right_avoidance_events;
  int replan_attempts;
  int significant_turns;
  double planned_distance_m;
  double executed_distance_m;
  double max_position_error_m;
  double final_position_error_m;
} MetricsState;

typedef struct {
  const ScenarioConfig *scenario;
  Pose2D odom_pose;
  Pose2D gps_reference;
  ProximityState proximity;
  EncoderState encoders;
  Punto planned_route[MAX_ROUTE_POINTS];
  int route_length;
  int current_waypoint;
  int step_counter;
  bool route_completed;
  MetricsState metrics;
} ControllerState;

static const ScenarioConfig SIMPLE_SCENARIO = {
  "simple",
  {3, 0},
  {3, 9},
  {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
    {0, 1, 0, 0, 0, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 0, 1, 0, 0, 0, 0},
    {0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  }
};

static const ScenarioConfig COMPLEX_SCENARIO = {
  "complex",
  {3, 0},
  {3, 9},
  {
    {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
    {0, 1, 1, 0, 0, 1, 0, 1, 0, 0},
    {0, 0, 1, 0, 1, 0, 0, 1, 0, 0},
    {0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
    {0, 1, 0, 0, 1, 0, 0, 0, 1, 0},
    {0, 0, 0, 1, 0, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  }
};

static double normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

static double clamp_speed(double speed) {
  if (speed > MAX_SPEED)
    return MAX_SPEED;
  if (speed < -MAX_SPEED)
    return -MAX_SPEED;
  return speed;
}

static bool is_cell_in_bounds(Punto p) {
  return p.fila >= 0 && p.fila < GRID_ROWS && p.col >= 0 && p.col < GRID_COLS;
}

static bool is_cell_blocked(const ScenarioConfig *scenario, Punto p) {
  return scenario->grid[p.fila][p.col] != 0;
}

static int point_to_index(Punto p) {
  return p.fila * GRID_COLS + p.col;
}

static Punto index_to_point(int index) {
  Punto p;
  p.fila = index / GRID_COLS;
  p.col = index % GRID_COLS;
  return p;
}

static int manhattan_distance(Punto a, Punto b) {
  return abs(a.fila - b.fila) + abs(a.col - b.col);
}

static void grid_to_world(Punto p, double *x, double *y) {
  *x = GRID_ORIGIN_X + (p.col * CELL_SIZE);
  *y = GRID_ORIGIN_Y + (p.fila * CELL_SIZE);
}

static Punto world_to_grid(double x, double y) {
  Punto p;
  p.col = (int)lround((x - GRID_ORIGIN_X) / CELL_SIZE);
  p.fila = (int)lround((y - GRID_ORIGIN_Y) / CELL_SIZE);

  if (p.col < 0)
    p.col = 0;
  if (p.col >= GRID_COLS)
    p.col = GRID_COLS - 1;
  if (p.fila < 0)
    p.fila = 0;
  if (p.fila >= GRID_ROWS)
    p.fila = GRID_ROWS - 1;
  return p;
}

static const ScenarioConfig *select_scenario(void) {
  const char *custom_data = wb_robot_get_custom_data();
  if (custom_data && strcmp(custom_data, "complex") == 0)
    return &COMPLEX_SCENARIO;
  return &SIMPLE_SCENARIO;
}

static void log_grid(const ScenarioConfig *scenario) {
  int row;
  int col;
  printf("Escenario activo: %s\n", scenario->name);
  printf("Inicio=(%d,%d) Meta=(%d,%d)\n", scenario->start.fila, scenario->start.col, scenario->goal.fila, scenario->goal.col);
  printf("Grilla:\n");
  for (row = 0; row < GRID_ROWS; ++row) {
    printf("  ");
    for (col = 0; col < GRID_COLS; ++col)
      printf("%d ", scenario->grid[row][col]);
    printf("\n");
  }
}

static double compute_planned_distance(const Punto *route, int route_length) {
  int i;
  double total = 0.0;
  for (i = 1; i < route_length; ++i)
    total += CELL_SIZE * hypot((double)(route[i].col - route[i - 1].col), (double)(route[i].fila - route[i - 1].fila));
  return total;
}

static void log_planned_route(const Punto *route, int route_length) {
  int i;
  printf("Ruta A* calculada con %d puntos:\n", route_length);
  for (i = 0; i < route_length; ++i)
    printf("  [%d] celda=(%d,%d)\n", i, route[i].fila, route[i].col);
}

static int reconstruct_path(AStarNode *nodes, int goal_index, Punto *route, int max_route_points) {
  Punto reversed_route[MAX_ROUTE_POINTS];
  int count = 0;
  int current = goal_index;
  int i;

  while (current != -1 && count < max_route_points) {
    reversed_route[count++] = index_to_point(current);
    current = nodes[current].parent_index;
  }

  for (i = 0; i < count; ++i)
    route[i] = reversed_route[count - 1 - i];

  return count;
}

static int plan_route_a_star(const ScenarioConfig *scenario, Punto start, Punto goal, Punto *route, int max_route_points) {
  AStarNode nodes[MAX_ROUTE_POINTS];
  const Punto neighbors[4] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  int start_index;
  int goal_index;
  int current_index;
  int i;

  if (!is_cell_in_bounds(start) || !is_cell_in_bounds(goal))
    return 0;
  if (is_cell_blocked(scenario, start) || is_cell_blocked(scenario, goal))
    return 0;

  for (i = 0; i < MAX_ROUTE_POINTS; ++i) {
    nodes[i].parent_index = -1;
    nodes[i].g_cost = LARGE_COST;
    nodes[i].f_cost = LARGE_COST;
    nodes[i].open = false;
    nodes[i].closed = false;
  }

  start_index = point_to_index(start);
  goal_index = point_to_index(goal);
  nodes[start_index].g_cost = 0;
  nodes[start_index].f_cost = manhattan_distance(start, goal);
  nodes[start_index].open = true;

  while (true) {
    int best_f_cost = LARGE_COST;
    current_index = -1;

    for (i = 0; i < MAX_ROUTE_POINTS; ++i) {
      if (nodes[i].open && nodes[i].f_cost < best_f_cost) {
        best_f_cost = nodes[i].f_cost;
        current_index = i;
      }
    }

    if (current_index == -1)
      return 0;

    if (current_index == goal_index)
      return reconstruct_path(nodes, goal_index, route, max_route_points);

    nodes[current_index].open = false;
    nodes[current_index].closed = true;

    {
      Punto current = index_to_point(current_index);
      int n;
      for (n = 0; n < 4; ++n) {
        Punto neighbor;
        int neighbor_index;
        int tentative_g_cost;

        neighbor.fila = current.fila + neighbors[n].fila;
        neighbor.col = current.col + neighbors[n].col;

        if (!is_cell_in_bounds(neighbor) || is_cell_blocked(scenario, neighbor))
          continue;

        neighbor_index = point_to_index(neighbor);
        if (nodes[neighbor_index].closed)
          continue;

        tentative_g_cost = nodes[current_index].g_cost + 1;
        if (!nodes[neighbor_index].open || tentative_g_cost < nodes[neighbor_index].g_cost) {
          nodes[neighbor_index].parent_index = current_index;
          nodes[neighbor_index].g_cost = tentative_g_cost;
          nodes[neighbor_index].f_cost = tentative_g_cost + manhattan_distance(neighbor, goal);
          nodes[neighbor_index].open = true;
        }
      }
    }
  }
}

static void read_proximity_sensors(const WbDeviceTag sensors[8], ProximityState *state) {
  int i;
  for (i = 0; i < 8; ++i)
    state->values[i] = wb_distance_sensor_get_value(sensors[i]);

  state->left = fmax(state->values[5], state->values[6]);
  state->right = fmax(state->values[1], state->values[2]);
  state->front = fmax(fmax(state->values[0], state->values[7]), state->values[6]);
}

static void read_encoders(WbDeviceTag left_sensor, WbDeviceTag right_sensor, EncoderState *state) {
  state->left = wb_position_sensor_get_value(left_sensor);
  state->right = wb_position_sensor_get_value(right_sensor);
}

static bool encoders_ready(const EncoderState *state) {
  return isfinite(state->left) && isfinite(state->right) && isfinite(state->prev_left) && isfinite(state->prev_right);
}

static void update_odometry(ControllerState *state) {
  double delta_left = (state->encoders.left - state->encoders.prev_left) * WHEEL_RADIUS;
  double delta_right = (state->encoders.right - state->encoders.prev_right) * WHEEL_RADIUS;
  double delta_s = (delta_left + delta_right) * 0.5;
  double delta_theta = (delta_right - delta_left) / AXLE_LENGTH;
  double heading_mid = state->odom_pose.theta + (delta_theta * 0.5);

  if (!isfinite(delta_left) || !isfinite(delta_right) || !isfinite(delta_s) || !isfinite(delta_theta) ||
      !isfinite(heading_mid))
    return;

  state->odom_pose.x += delta_s * cos(heading_mid);
  state->odom_pose.y += delta_s * sin(heading_mid);
  state->odom_pose.theta = normalize_angle(state->odom_pose.theta + delta_theta);
  state->metrics.executed_distance_m += fabs(delta_s);

  state->encoders.prev_left = state->encoders.left;
  state->encoders.prev_right = state->encoders.right;
}

static double compute_heading_error(const Pose2D *pose, double target_x, double target_y) {
  double desired_heading = atan2(target_y - pose->y, target_x - pose->x);
  if (!isfinite(desired_heading) || !isfinite(pose->theta))
    return 0.0;
  return normalize_angle(desired_heading - pose->theta);
}

static void apply_motor_command(WbDeviceTag left_motor, WbDeviceTag right_motor, double left_speed, double right_speed) {
  if (!isfinite(left_speed))
    left_speed = 0.0;
  if (!isfinite(right_speed))
    right_speed = 0.0;
  wb_motor_set_velocity(left_motor, clamp_speed(left_speed));
  wb_motor_set_velocity(right_motor, clamp_speed(right_speed));
}

static bool handle_local_avoidance(ControllerState *state, double heading_error, WbDeviceTag left_motor,
                                   WbDeviceTag right_motor) {
  const ProximityState *proximity = &state->proximity;

  if (proximity->front > SENSOR_THRESHOLD) {
    double turn_direction = heading_error >= 0.0 ? 1.0 : -1.0;
    state->metrics.avoidance_events++;
    state->metrics.front_avoidance_events++;

    if (proximity->left + 5.0 < proximity->right)
      turn_direction = 1.0;
    else if (proximity->right + 5.0 < proximity->left)
      turn_direction = -1.0;

    apply_motor_command(left_motor, right_motor, -turn_direction * TURN_SPEED, turn_direction * TURN_SPEED);
    return true;
  }

  if (proximity->left > SENSOR_THRESHOLD) {
    state->metrics.avoidance_events++;
    state->metrics.left_avoidance_events++;
    apply_motor_command(left_motor, right_motor, CRUISE_SPEED, 0.45 * CRUISE_SPEED);
    return true;
  }

  if (proximity->right > SENSOR_THRESHOLD) {
    state->metrics.avoidance_events++;
    state->metrics.right_avoidance_events++;
    apply_motor_command(left_motor, right_motor, 0.45 * CRUISE_SPEED, CRUISE_SPEED);
    return true;
  }

  return false;
}

static void follow_waypoint(ControllerState *state, double target_x, double target_y, WbDeviceTag left_motor,
                            WbDeviceTag right_motor) {
  double distance = hypot(target_x - state->odom_pose.x, target_y - state->odom_pose.y);
  double heading_error = compute_heading_error(&state->odom_pose, target_x, target_y);
  double base_speed = CRUISE_SPEED;
  double correction = 2.2 * heading_error;
  double left_speed;
  double right_speed;

  if (distance < 0.45)
    base_speed *= fmax(0.35, distance / 0.45);

  if (fabs(heading_error) > 0.4) {
    state->metrics.significant_turns++;
    left_speed = -TURN_SPEED;
    right_speed = TURN_SPEED;
    if (heading_error < 0.0) {
      left_speed = TURN_SPEED;
      right_speed = -TURN_SPEED;
    }
  } else {
    left_speed = base_speed - correction;
    right_speed = base_speed + correction;
  }

  apply_motor_command(left_motor, right_motor, left_speed, right_speed);
}

static void update_position_error(ControllerState *state) {
  double error = hypot(state->gps_reference.x - state->odom_pose.x, state->gps_reference.y - state->odom_pose.y);
  state->metrics.final_position_error_m = error;
  if (error > state->metrics.max_position_error_m)
    state->metrics.max_position_error_m = error;
}

static void log_state(const ControllerState *state, const double *gps_values, const double *compass_values) {
  Punto current_cell = world_to_grid(state->odom_pose.x, state->odom_pose.y);
  double compass_heading = normalize_angle(atan2(compass_values[0], compass_values[1]));

  printf("escenario=%s odom=(%.2f, %.2f, %.2f) gps_ref=(%.2f, %.2f) compass=%.2f cell=(%d,%d) waypoint=%d/%d avoid=%d err=%.2f\n",
         state->scenario->name, state->odom_pose.x, state->odom_pose.y, state->odom_pose.theta, gps_values[0],
         gps_values[1], compass_heading, current_cell.fila, current_cell.col, state->current_waypoint,
         state->route_length - 1, state->metrics.avoidance_events, state->metrics.final_position_error_m);
}

static void log_metrics_summary(const ControllerState *state) {
  double simulated_seconds = state->step_counter * (TIME_STEP / 1000.0);
  printf("=== Resumen de metricas (%s) ===\n", state->scenario->name);
  printf("estado_final=%s\n", state->route_completed ? "exito" : "incompleto");
  printf("tiempo_simulado_s=%.2f\n", simulated_seconds);
  printf("ruta_planificada_celdas=%d\n", state->route_length);
  printf("longitud_ruta_planificada_m=%.2f\n", state->metrics.planned_distance_m);
  printf("longitud_trayectoria_ejecutada_m=%.2f\n", state->metrics.executed_distance_m);
  printf("eventos_evasion_total=%d\n", state->metrics.avoidance_events);
  printf("eventos_evasion_frontal=%d\n", state->metrics.front_avoidance_events);
  printf("eventos_evasion_izquierda=%d\n", state->metrics.left_avoidance_events);
  printf("eventos_evasion_derecha=%d\n", state->metrics.right_avoidance_events);
  printf("giros_significativos=%d\n", state->metrics.significant_turns);
  printf("replanificaciones=%d\n", state->metrics.replan_attempts);
  printf("error_posicion_final_m=%.3f\n", state->metrics.final_position_error_m);
  printf("error_posicion_max_m=%.3f\n", state->metrics.max_position_error_m);
}

int main(void) {
  WbDeviceTag left_motor;
  WbDeviceTag right_motor;
  WbDeviceTag gps;
  WbDeviceTag compass;
  WbDeviceTag left_position_sensor;
  WbDeviceTag right_position_sensor;
  WbDeviceTag proximity_sensors[8];
  ControllerState state;
  double start_x;
  double start_y;
  int i;

  memset(&state, 0, sizeof(state));
  wb_robot_init();

  state.scenario = select_scenario();
  log_grid(state.scenario);

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  gps = wb_robot_get_device("gps");
  compass = wb_robot_get_device("compass");
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");

  wb_gps_enable(gps, TIME_STEP);
  wb_compass_enable(compass, TIME_STEP);
  wb_position_sensor_enable(left_position_sensor, TIME_STEP);
  wb_position_sensor_enable(right_position_sensor, TIME_STEP);

  for (i = 0; i < 8; ++i) {
    char name[4];
    sprintf(name, "ps%d", i);
    proximity_sensors[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(proximity_sensors[i], TIME_STEP);
  }

  grid_to_world(state.scenario->start, &start_x, &start_y);
  state.odom_pose.x = start_x;
  state.odom_pose.y = start_y;
  state.odom_pose.theta = 0.0;
  state.gps_reference.x = start_x;
  state.gps_reference.y = start_y;
  state.gps_reference.theta = 0.0;
  state.route_length = plan_route_a_star(state.scenario, state.scenario->start, state.scenario->goal,
                                         state.planned_route, MAX_ROUTE_POINTS);
  state.current_waypoint = 0;

  if (state.route_length <= 0) {
    fprintf(stderr, "No se pudo calcular una ruta A* valida para el escenario %s.\n", state.scenario->name);
    wb_robot_cleanup();
    return 1;
  }

  state.metrics.planned_distance_m = compute_planned_distance(state.planned_route, state.route_length);
  log_planned_route(state.planned_route, state.route_length);

  state.encoders.left = NAN;
  state.encoders.right = NAN;
  state.encoders.prev_left = NAN;
  state.encoders.prev_right = NAN;

  while (wb_robot_step(TIME_STEP) != -1) {
    const double *gps_values;
    const double *compass_values;
    Punto target_cell;
    double target_x;
    double target_y;
    double heading_error;

    read_proximity_sensors(proximity_sensors, &state.proximity);
    read_encoders(left_position_sensor, right_position_sensor, &state.encoders);

    if (!isfinite(state.encoders.prev_left) || !isfinite(state.encoders.prev_right)) {
      if (!isfinite(state.encoders.left) || !isfinite(state.encoders.right)) {
        apply_motor_command(left_motor, right_motor, 0.0, 0.0);
        continue;
      }
      state.encoders.prev_left = state.encoders.left;
      state.encoders.prev_right = state.encoders.right;
    }

    if (!encoders_ready(&state.encoders)) {
      apply_motor_command(left_motor, right_motor, 0.0, 0.0);
      continue;
    }

    update_odometry(&state);

    gps_values = wb_gps_get_values(gps);
    compass_values = wb_compass_get_values(compass);
    state.gps_reference.x = gps_values[0];
    state.gps_reference.y = gps_values[1];
    update_position_error(&state);

    if (state.current_waypoint >= state.route_length) {
      if (!state.route_completed) {
        state.route_completed = true;
        apply_motor_command(left_motor, right_motor, 0.0, 0.0);
        printf("Ruta A* completada.\n");
        log_metrics_summary(&state);
      }
      continue;
    }

    target_cell = state.planned_route[state.current_waypoint];
    grid_to_world(target_cell, &target_x, &target_y);

    if (hypot(target_x - state.odom_pose.x, target_y - state.odom_pose.y) < WAYPOINT_TOLERANCE) {
      state.current_waypoint++;
      if (state.current_waypoint >= state.route_length)
        continue;
      target_cell = state.planned_route[state.current_waypoint];
      grid_to_world(target_cell, &target_x, &target_y);
    }

    heading_error = compute_heading_error(&state.odom_pose, target_x, target_y);
    if (!handle_local_avoidance(&state, heading_error, left_motor, right_motor))
      follow_waypoint(&state, target_x, target_y, left_motor, right_motor);

    if ((state.step_counter % LOG_INTERVAL_STEPS) == 0)
      log_state(&state, gps_values, compass_values);

    state.step_counter++;
  }

  if (!state.route_completed)
    log_metrics_summary(&state);

  wb_robot_cleanup();
  return 0;
}
