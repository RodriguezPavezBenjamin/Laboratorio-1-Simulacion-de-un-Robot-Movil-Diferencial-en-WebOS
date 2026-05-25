# Laboratorio 2: Navegación reactiva con filtrado y fusión de sensores en Webots

**Curso:** Robótica y Sistemas Autónomos 2026-01 — ICI 4150  
**Robot:** E-puck (diferencial de dos ruedas)  
**Simulador:** Webots R2025a

---

# Contenido
1. [Descripción del Laboratorio](#descripción-del-laboratorio)
2. [Sensores de Distancia (E-puck)](#sensores-de-distancia-e-puck)
3. [Encoders de Rueda](#encoders-de-rueda)
4. [Filtrado de Señales](#filtrado-de-señales)
5. [Filtro de Kalman Escalar](#filtro-de-kalman-escalar)
6. [Arquitectura de Control Reactivo](#arquitectura-de-control-reactivo)
7. [Algoritmo Principal del Controlador](#algoritmo-principal-del-controlador)

---

## Descripción del Laboratorio

**Curso:** Robótica y Sistemas Autónomos 2026-01 — ICI 4150  
**Robot:** E-puck (diferencial de dos ruedas)  
**Simulador:** Webots R2025a

Se implementa un sistema de navegación reactiva para un robot móvil diferencial, combinando sensores infrarrojos y encoders de rueda. Se aplica un Filtro de Kalman escalar para estimar la distancia frontal a obstáculos de forma robusta, mejorando la toma de decisiones en tiempo real.

---

## Sensores de Distancia (E-puck)

| Sensor  | Ubicación       | Función                        |
| ------- | --------------- | ------------------------------ |
| `ps0`   | Frontal derecho | Detección frontal              |
| `ps1`   | Diagonal derecho| Deslizamiento por bordes       |
| `ps2`   | Lateral derecho | Decisión de giro               |
| `ps5`   | Lateral izquierdo | Decisión de giro             |
| `ps6`   | Diagonal izquierdo | Deslizamiento por bordes    |
| `ps7`   | Frontal izquierdo | Detección frontal            |

Los sensores infrarrojos entregan un valor analógico que debe convertirse a metros con la siguiente función de calibración:

```c
float prox_to_meters(float val) {
    if (val < 80)   return 0.06;   // Sin obstáculo: máximo rango
    if (val > 3500) return 0.01;   // Obstáculo muy cercano: mínimo rango
    return max(0.01, min(-0.0132 * log(val) + 0.1178, 0.06));
}
```

*A tener en cuenta:*
1. El rango útil del sensor va de **1 cm a 6 cm**.
2. La respuesta es no lineal: se aplica una corrección logarítmica.
3. Valores fuera del rango se truncan al mínimo o máximo según corresponda.

----

## Encoders de Rueda

| Sensor              | Pin Webots             | Función                        |
| ------------------- | ---------------------- | ------------------------------ |
| Encoder izquierdo   | `left wheel sensor`    | Desplazamiento rueda izquierda |
| Encoder derecho     | `right wheel sensor`   | Desplazamiento rueda derecha   |

Los encoders entregan el desplazamiento angular acumulado en radianes. Para obtener el avance lineal del robot:

```python
RADIO_RUEDA_METROS = 0.0205  # Radio del e-puck en metros

delta_th_izq = left_enc  - anterior_izq
delta_th_der = right_enc - anterior_der

# Avance lineal neto del robot
delta_robot = (RADIO_RUEDA_METROS * delta_th_izq + RADIO_RUEDA_METROS * delta_th_der) / 2.0
```

**¿Qué es Calibrar los Encoders?**  
Verificar que el radio de rueda empleado en la fórmula coincida con el valor físico real, ya que un error en ese parámetro acumula deriva en la estimación de posición.

*Pasos de calibración:*
1. Marca una posición inicial en el suelo y registra el valor del encoder.
2. Desplaza el robot una distancia conocida (por ejemplo, 10 cm).
3. Compara el desplazamiento calculado con la distancia real.
4. Ajusta `RADIO_RUEDA_METROS` según el error observado.

----

## Filtrado de Señales

En la práctica, los sensores infrarrojos presentan ruido y respuestas bruscas. Se implementaron dos métodos de filtrado sobre la lectura frontal mínima (`z_frontal_cruda`).

### Filtro Simple (Exponencial / Alfa)

Suaviza la señal cruda introduciendo un retardo controlado por el factor `α`:

```python
alpha = 0.3  # Factor de suavizado (0 = sin cambios, 1 = sin filtro)

z_filtrada = alpha * z_cruda + (1 - alpha) * z_filtrada_anterior
```

**¿Qué es el factor α?**  
Es el peso que se le da a la nueva medición. Un `α` bajo produce mayor suavizado pero más retardo; un `α` alto sigue mejor la señal pero filtra menos ruido.

*A tener en cuenta:*
- Un `α = 0.3` introduce un retardo perceptible ante cambios rápidos.
- Este retardo **no afecta** la seguridad porque la decisión de evasión se basa en el Filtro de Kalman, no en esta señal.

----

## Filtro de Kalman Escalar

Combina la predicción de movimiento (encoders) con la percepción del entorno (sensor frontal) para estimar la distancia de forma anticipada y estable.

**Parámetros iniciales**

| Parámetro                  | Variable                      | Valor |
| -------------------------- | ----------------------------- | ----- |
| Distancia estimada inicial | `distancia_estimada_kalman`   | 0.06 m |
| Covarianza inicial         | `covarianza_error_estimacion` | 1.0   |
| Ruido de proceso (encoders)| `ruido_proceso_encoders`      | 0.005 |
| Ruido de medición (sensor) | `ruido_medicion_sensor`       | 0.002 |

### Etapa de Predicción

Se estima la distancia futura restando el avance del robot al estado anterior:

```python
d_pred = distancia_estimada_kalman - delta_robot
P_pred = covarianza_error_estimacion + ruido_proceso_encoders
```

### Etapa de Corrección

Se ajusta la predicción con la medición del sensor usando la Ganancia de Kalman:

```python
ganancia_kalman = P_pred / (P_pred + ruido_medicion_sensor)

distancia_estimada_kalman = d_pred + ganancia_kalman * (z_frontal_cruda - d_pred)
covarianza_error_estimacion = (1 - ganancia_kalman) * P_pred
```

**¿Qué es la Ganancia de Kalman?**  
Es un valor entre 0 y 1 que pondera cuánto se confía en el sensor versus en la predicción. Si la covarianza es alta (predicción incierta), la ganancia sube y el filtro confía más en el sensor.

*A tener en cuenta:*
1. Cuando ocurre un reflejo de pánico, se reinicia la covarianza a `1.0` para forzar al filtro a confiar en el sensor.
2. La estimación de Kalman es más reactiva que el filtro exponencial y más estable que la señal cruda.

----

## Arquitectura de Control Reactivo

La toma de decisiones se implementó como un sistema de **control por capas**, evaluado en orden de prioridad en cada paso de simulación:

| Capa | Nombre                  | Condición de activación              | Acción                              |
| ---- | ----------------------- | ------------------------------------ | ----------------------------------- |
| 0    | Reflejo de Pánico       | `z_cruda ≤ 3.5 cm`                   | Giro forzado, reinicia covarianza   |
| 1    | Memoria de Maniobra     | `pasos_giro_restantes > 0`           | Continúa el giro en curso           |
| 2    | Evasión por Kalman      | `d_kalman ≤ 5.0 cm`                  | Inicia giro anticipado              |
| 3    | Deslizamiento Diagonal  | Sensor diagonal `≤ 5.5 cm`          | Curva suave sin detener el avance   |
| 4    | Camino Libre            | Ninguna condición anterior activa    | Avance recto a velocidad crucero    |

**¿Qué es un Umbral?**  
Es el valor límite a partir del cual una capa se activa. Por ejemplo, con un umbral de seguridad de 5 cm: *si la distancia estimada es menor a 5 cm, el robot debe evadir*.

```python
UMBRAL_SEGURIDAD_METROS   = 0.05   # Capa 2: evasión por Kalman
UMBRAL_DIAGONALES_METROS  = 0.055  # Capa 3: deslizamiento suave
UMBRAL_PANICO_METROS      = 0.035  # Capa 0: evasión de emergencia
```

----

## Algoritmo Principal del Controlador

```python
import math
from controller import Robot, DistanceSensor, Motor

# --- PARÁMETROS ---
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())
RADIO_RUEDA_METROS   = 0.0205
VELOCIDAD_MAX        = 6.28
UMBRAL_SEGURIDAD     = 0.05
UMBRAL_DIAGONALES    = 0.055
UMBRAL_PANICO        = 0.035

# --- INICIALIZACIÓN DE MOTORES ---
motor_izq = robot.getDevice('left wheel motor')
motor_der = robot.getDevice('right wheel motor')
motor_izq.setPosition(float('inf'))
motor_der.setPosition(float('inf'))

# --- SENSORES ---
sensores = {n: robot.getDevice(n) for n in ['ps0','ps1','ps2','ps5','ps6','ps7']}
for s in sensores.values(): s.enable(TIME_STEP)

enc_izq = robot.getDevice('left wheel sensor');  enc_izq.enable(TIME_STEP)
enc_der = robot.getDevice('right wheel sensor'); enc_der.enable(TIME_STEP)

# --- ESTADO KALMAN ---
d_kalman = 0.06
P        = 1.0
Q        = 0.005  # Ruido de proceso
R        = 0.002  # Ruido de medición

pasos_giro = 0

while robot.step(TIME_STEP) != -1:

    # Lectura de sensores
    z_cruda    = min(prox_to_meters(sensores['ps0'].getValue()),
                     prox_to_meters(sensores['ps7'].getValue()))
    diag_der   = prox_to_meters(sensores['ps1'].getValue())
    diag_izq   = prox_to_meters(sensores['ps6'].getValue())
    lat_izq    = sensores['ps5'].getValue()
    lat_der    = sensores['ps2'].getValue()

    # Avance por encoders
    delta = (RADIO_RUEDA_METROS * (enc_izq.getValue() - ant_izq) +
             RADIO_RUEDA_METROS * (enc_der.getValue() - ant_der)) / 2.0

    # Filtro de Kalman
    d_pred   = d_kalman - delta
    P_pred   = P + Q
    K        = P_pred / (P_pred + R)
    d_kalman = d_pred + K * (z_cruda - d_pred)
    P        = (1 - K) * P_pred

    # Dirección de giro (lado con más espacio)
    if pasos_giro == 0:
        vel_izq_ev = VELOCIDAD_MAX * 0.5  if lat_izq > lat_der else -VELOCIDAD_MAX * 0.5
        vel_der_ev = -VELOCIDAD_MAX * 0.5 if lat_izq > lat_der else  VELOCIDAD_MAX * 0.5

    # --- CAPAS DE CONTROL ---
    if z_cruda <= UMBRAL_PANICO and pasos_giro <= 5:       # Capa 0
        pasos_giro = 25; P = 1.0
        vi, vd = vel_izq_ev, vel_der_ev

    elif pasos_giro > 0:                                    # Capa 1
        pasos_giro -= 1; P = 1.0
        vi, vd = vel_izq_ev, vel_der_ev

    elif d_kalman <= UMBRAL_SEGURIDAD:                      # Capa 2
        pasos_giro = 18
        vi, vd = vel_izq_ev, vel_der_ev

    elif diag_izq <= UMBRAL_DIAGONALES:                     # Capa 3
        vi, vd = VELOCIDAD_MAX * 0.4, VELOCIDAD_MAX * 0.05

    elif diag_der <= UMBRAL_DIAGONALES:                     # Capa 3
        vi, vd = VELOCIDAD_MAX * 0.05, VELOCIDAD_MAX * 0.4

    else:                                                   # Capa 4
        vi, vd = VELOCIDAD_MAX * 0.4, VELOCIDAD_MAX * 0.4

    motor_izq.setVelocity(vi)
    motor_der.setVelocity(vd)
```

### Cómo ejecutar la simulación

**Requisitos previos**
- [Webots R2025a](https://cyberbotics.com/) instalado
- Python 3.x

**Pasos**

1. Clona el repositorio:
   ```bash
   git clone <URL_DEL_REPOSITORIO>
   cd <NOMBRE_DEL_REPOSITORIO>
   ```

2. Abre Webots y carga el mundo:
   - Ve a **File → Open World**
   - Selecciona `worlds/Laboratorio2.wbt` (escenario fácil) o `worlds/Laboratorio2_EscComplejo.wbt` (escenario complejo)

3. Ejecuta la simulación:
   - Abre el archivo `epuck_Lab2.py` en el editor de controladores
   - Presiona **Play ▶**

---

## Resultados

| Escenario | Duración | Maniobras de Evasión | Colisiones |
| --------- | -------- | -------------------- | ---------- |
| Simple    | 60 s     | 9                    | 0          |
| Complejo  | 118 s    | 22                   | 0          |

---

## Integrantes

| Rol           | Nombre              |
| ------------- | ------------------- |
| Programador   | Sebastian Rojas     |
| Experimentador| Ignacio Layana      |
| Analista      | Benjamin Rodriguez  |
| Documentador  | Benjamin Rodriguez  |
| Integrador    | Javier Morales      |
