/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// ===========================================================================================
// Incluye la librería principal para inicializar y controlar el robot en Webots.
#include <webots/robot.h>

// Incluye la librería específica para poder usar y dar instrucciones a los motores.
#include <webots/motor.h>

// Define el "paso de tiempo" (TIME_STEP) de la simulación en milisegundos.
// Es decir, cada 64ms este código se comunicará con el simulador para actualizar el estado del robot.
#define TIME_STEP 64

// Define una constante con la velocidad máxima a la que pueden girar los motores.
// En Webots, la velocidad se mide en radianes por segundo.
#define MAX_SPEED 6.28

int main(int argc, char **argv) {
  // Inicializa la comunicación entre tu código (el controlador) y el simulador Webots.
  wb_robot_init();

  // Busca y guarda los identificadores de los motores izquierdo y derecho 
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  
  // Configura los motores para control de velocidad constante (girar infinitamente).
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  // Establece la velocidad de los motores.
  wb_motor_set_velocity(left_motor, 0.5 * MAX_SPEED);
  wb_motor_set_velocity(right_motor, (0.5*0.5) * MAX_SPEED);

  // Ejecución del robot
  while (wb_robot_step(TIME_STEP) != -1) {
  }

  wb_robot_cleanup();

  return 0;
}
