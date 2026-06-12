#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define TAMANO_CASILLA 1.03
#define OFFSET_X -5.14
#define OFFSET_Y -3.63

typedef struct { int fila; int col; } Punto;

int cancha_grid[7][10] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
    {0, 1, 0, 0, 0, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 0, 1, 0, 0, 0, 0},
    {0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

Punto ruta_calculada[] = {{3,1}, {3,2}, {2,2}, {2,3}, {2,4}, {3,4}, {3,5}, {3,6}, {4,6}, {4,7}, {4,8}, {3,8}, {3,9}};
int total_pasos = 13;
int paso_actual = 0;

void obtener_coordenadas_fisicas(Punto p, double *x, double *y) {
    *x = (p.col * TAMANO_CASILLA) + OFFSET_X; 
    *y = (p.fila * TAMANO_CASILLA) + OFFSET_Y; 
}

double obtener_angulo_actual(WbDeviceTag compass) {
    const double *v = wb_compass_get_values(compass);
    return atan2(v[0], v[2]);
}

int main() {
    wb_robot_init();

    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    WbDeviceTag gps = wb_robot_get_device("gps");
    WbDeviceTag compass = wb_robot_get_device("compass");
    wb_gps_enable(gps, TIME_STEP);
    wb_compass_enable(compass, TIME_STEP);

    WbDeviceTag ps[8];
    char name[4];
    for (int i = 0; i < 8; i++) {
        sprintf(name, "ps%d", i);
        ps[i] = wb_robot_get_device(name);
        wb_distance_sensor_enable(ps[i], TIME_STEP);
    }

    while (wb_robot_step(TIME_STEP) != -1) {
        if (paso_actual >= total_pasos) {
            wb_motor_set_velocity(left_motor, 0.0);
            wb_motor_set_velocity(right_motor, 0.0);
            continue;
        }

        bool izq = wb_distance_sensor_get_value(ps[5]) > 80.0 || wb_distance_sensor_get_value(ps[6]) > 80.0;
        bool der = wb_distance_sensor_get_value(ps[1]) > 80.0 || wb_distance_sensor_get_value(ps[2]) > 80.0;
        bool frente = wb_distance_sensor_get_value(ps[0]) > 80.0 || wb_distance_sensor_get_value(ps[7]) > 80.0;

        if (frente || izq || der) {
            wb_motor_set_velocity(left_motor, izq ? 0.5 * MAX_SPEED : -0.5 * MAX_SPEED);
            wb_motor_set_velocity(right_motor, izq ? -0.5 * MAX_SPEED : 0.5 * MAX_SPEED);
            continue;
        }

        const double *pos = wb_gps_get_values(gps);
        double target_x, target_y;
        obtener_coordenadas_fisicas(ruta_calculada[paso_actual], &target_x, &target_y);

        double dx = target_x - pos[0];
        double dy = target_y - pos[1]; 
        
        if (sqrt(dx * dx + dy * dy) < 0.15) { 
            paso_actual++;
            continue;
        }

        double angulo_objetivo = atan2(dy, dx);
        double error = angulo_objetivo - obtener_angulo_actual(compass);
        
        while (error > M_PI) error -= 2.0 * M_PI;
        while (error < -M_PI) error += 2.0 * M_PI;

        double v_l = MAX_SPEED * 0.8;
        double v_r = MAX_SPEED * 0.8;

        if (error > 0.1) v_r *= 0.5;
        else if (error < -0.1) v_l *= 0.5;

        wb_motor_set_velocity(left_motor, v_l);
        wb_motor_set_velocity(right_motor, v_r);
    }

    wb_robot_cleanup();
    return 0;
}