# Laboratorio 2: Navegación reactiva con filtrado y fusión de sensores en Webots

**Curso:** Robótica y Sistemas Autónomos 2026-01 — ICI 4150  
**Robot:** E-puck (diferencial de dos ruedas)  
**Simulador:** Webots R2025a

---

## 1. Objetivo del trabajo

Implementar un sistema básico de navegación reactiva en el simulador Webots para un robot móvil diferencial, utilizando sensores de distancia y encoders de rueda. Se aplican técnicas de filtrado de señales y un Filtro de Kalman escalar para estimar de forma robusta la distancia frontal hacia los obstáculos, mejorando así la toma de decisiones del robot en tiempo real

---

## 2. Descripción del laboratorio
Para este laboratorio se utilizó el robot **e-puck**, el cual opera como un sistema diferencial con dos ruedas motrices independientes. La percepción del entorno se resolvió empleando los siguientes sensores:
* **Sensores de Distancia (Infrarrojos):** Se habilitaron 6 sensores (`ps0`, `ps1`, `ps2`, `ps5`, `ps6`, `ps7`) para detectar obstáculos. Aunque el mínimo requerido era de dos frontales y dos laterales, se incluyeron sensores diagonales para mejorar la evasión por deslizamiento.
* **Encoders de Rueda:** Sensores en la rueda izquierda y derecha (`left wheel sensor`, `right wheel sensor`) para estimar el avance lineal y el giro del robot a partir de la cinemática.

---

## 3. Frecuencia de Muestreo Empleada
Las lecturas de los sensores y encoders se registraron de forma síncrona utilizando el paso de simulación básico de Webots (`TIME_STEP`). 
* **Tiempo de muestreo:** $T_{s} =$ [Ej: 0.032] segundos.
* **Frecuencia de muestreo:** $f_{s} = \frac{1}{T_{s}} =$ [Ej: 31.25] Hz.
* **Muestras registradas:** [Ej: 4500] muestras por experimento, recolectadas mediante el sistema de guardado analítico del controlador.

---

## 4. Análisis de las Señales Registradas
En la práctica, los sensores infrarrojos presentan ruido, incertidumbre y respuestas no lineales ante diferentes superficies. Al analizar la señal cruda, se observa que la distancia leída fluctúa bruscamente frente a objetos cercanos, lo que provocaría un comportamiento errático (oscilaciones) si el robot tomara decisiones basándose únicamente en esta lectura instantánea.

---

## 5. Estimación del Avance mediante Encoders
Los encoders del e-puck entregan mediciones del desplazamiento angular en radianes. Para estimar el avance lineal del robot entre dos instantes de tiempo, calculamos el diferencial angular $\Delta \theta$ de cada rueda y lo convertimos a desplazamiento físico usando el radio de la rueda ($r = \text{0.0205 m}$), mediante la relación:
$$s = r\theta$$
El avance neto del centro geométrico del robot en su eje longitudinal se estimó promediando el desplazamiento lineal de ambas ruedas.

---

## 6. Filtrado y Fusión Sensorial (Filtro de Kalman)
Para obtener una representación confiable de la distancia frontal al obstáculo, se implementaron dos métodos:

### 6.1 Filtro Simple
Se aplicó un filtro paso bajo exponencial (Filtro Alfa) sobre el mínimo de las lecturas frontales, suavizando la señal cruda, pero introduciendo un inevitable retraso (lag) en la detección.

### 6.2 Filtro de Kalman
Se implementó un esquema de estimación para combinar la predicción de movimiento con la percepción del entorno.
* **Etapa de Predicción:** Se estimó el valor futuro de la distancia frontal restando el avance calculado por los encoders al estado anterior. Matemáticamente, se basa en la actualización del estado:   $$\hat{d}_{k}^{-} = \hat{d}_{k-1} - \Delta d_{k}$$  $\hat{d}_{k}^{-} = \hat{d}_{k-1} - \Delta d_{k}$
* **Etapa de Corrección:** La predicción se ajustó utilizando la medición cruda del sensor frontal ($z_k$). El peso de esta corrección lo dictó la Ganancia de Kalman ($K_k$), la cual se calcula dinámicamente evaluando la covarianza de la predicción ($P_k^-$) frente a la varianza del ruido del sensor ($R$): $$K_{k} = \frac{P_{k}^{-}}{P_{k}^{-} + R}$$
  
## 7. Lógica de Navegación Reactiva Implementada
La toma de decisiones se estructuró mediante una arquitectura de control por capas basándose en la distancia frontal estimada:
1. **Reflejo de Pánico (Capa 0):** Evasión de emergencia si la señal cruda detecta un impacto inminente (< 3.5 cm), forzando un reinicio de la covarianza del filtro.
2. **Memoria de Maniobra (Capa 1):** Bloqueo de giro persistente para evitar oscilaciones (efecto ping-pong) en las esquinas.
3. **Evasión por Kalman (Capa 2):** Si la estimación del filtro cruza el umbral de seguridad, el robot gira. La dirección de giro se decide evaluando qué sensor lateral tiene mayor espacio libre.
4. **Deslizamiento (Capa 3):** Uso de sensores diagonales para realizar curvas suaves al rozar obstáculos sin detener el avance.
5. **Camino Libre (Capa 4):** Si la estimación es mayor al umbral de seguridad, el robot avanza en línea recta a velocidad crucero.

---

## Experimentos realizados

FALTA REALIZAR EXPERIMENTOS

---

## 9. Resultados en Escenarios de Prueba
Se diseñaron dos entornos en Webots:
1. **Escenario Simple:** Entorno de ajedrez con tres obstáculos cúbicos de madera. El robot logró evadirlos fluidamente sin colisiones, estabilizando su línea de marcha rápidamente tras cada maniobra.
2. **Escenario Complejo:** Laberinto estrecho construido con cajas y elementos curvos. La implementación de la "Capa de Deslizamiento" y la "Memoria de Giro" demostraron ser vitales, permitiendo al robot sortear pasillos ciegos y vueltas de 90° sin atascarse en mínimos locales.

## 10. Conclusiones
FALTA AGREGAR CONCLUSIONES SEGÚN EL EXPERIMENTO

---

## Estructura del repositorio

```
├── worlds/
│   └── Laboratorio2.wbt                      # Mundo de simulación en Webots (Escenario Fácil)
│   └── Laboratorio2_EscenarioComplejo.wbt    # Mundo de simulación en Webots (Escenario Complejo con más obstáculos)
├── controllers/
│   └── epuck_Lab2/
│       └── epuck_Lab2.py      # Controlador del robot (Python)
├── screenshots/               # Capturas de pantalla de los experimentos
└── README.md
```

## Cómo ejecutar la simulación en Webots
### Requisitos previos

- [Webots R2025a](https://cyberbotics.com/) instalado
- Python 3.x

### Pasos

1. Clonar este repositorio en alguna carpeta de su dispositivo:
   ```bash
   git clone <URL_DEL_REPOSITORIO>
   cd <NOMBRE_DEL_REPOSITORIO>
   ```

2. Abrir Webots y cargar el mundo clonado:
   - Ir a **File → Open World**
   - Seleccionar el archivo `worlds/Laboratorio2.wbt`
   - Luego al haber probado con ese archivo, abrir para ejecutar con el escenario complejo `worlds/Laboratorio2_EscComplejo.wbt`

3. Ejecutar la simulación:
   - Seleccionar el archivo de texto `epuck_Lab2.py` para poder observar el comportamiento del robot
   - Presionar el botón **Play ▶** en Webots
   - El robot comenzará a moverse según el controlador activo

---

## Integrantes

| Rol | Nombre |
|---|---|
| Programador | Sebastian Rojas |
| Experimentador | Ignacio Layana |
| Analista | Benjamin Rodriguez |
| Documentador | Benjamin Rodriguez |
| Integrador |Javier Morales |
