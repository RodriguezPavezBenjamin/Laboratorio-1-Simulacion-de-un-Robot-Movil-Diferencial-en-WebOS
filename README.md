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
* **Tiempo de muestreo:** $T_{s} =$  0.016 segundos (16 ms).
* **Frecuencia de muestreo:** $f_{s} = \frac{1}{T_{s}} =$ 62.5 Hz.
* **Muestras registradas:** Se recolectaron datos durante toda la ejecución de las pruebas, resultando en:
  * **~3,750 muestras** para el Escenario Simple (60 segundos app de prueba).
  * **~7,375 muestras** para el Escenario Complejo (120 segundos app de prueba).

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

* **Etapa de Predicción:** Se estimó el valor futuro de la distancia frontal restando el avance calculado por los encoders al estado anterior. Matemáticamente, se basa en la actualización del estado:

$$\hat{d}_{k}^{-} = \hat{d}_{k-1} - \Delta d_{k}$$

* **Etapa de Corrección:** La predicción se ajustó utilizando la medición cruda del sensor frontal ($z_k$). El peso de esta corrección lo dictó la Ganancia de Kalman ($K_k$), la cual se calcula dinámicamente evaluando la covarianza de la predicción ($P_k^-$) frente a la varianza del ruido del sensor ($R$):

$$K_{k} = \frac{P_{k}^{-}}{P_{k}^{-} + R}$$

$$\hat{d}_{k} = \hat{d}_{k}^{-} + K_{k}(z_{k} - \hat{d}_{k}^{-})$$
  
## 7. Lógica de Navegación Reactiva Implementada
La toma de decisiones se estructuró mediante una arquitectura de control por capas basándose en la distancia frontal estimada:
1. **Reflejo de Pánico (Capa 0):** Evasión de emergencia si la señal cruda detecta un impacto inminente (< 3.5 cm), forzando un reinicio de la covarianza del filtro.
2. **Memoria de Maniobra (Capa 1):** Bloqueo de giro persistente para evitar oscilaciones (efecto ping-pong) en las esquinas.
3. **Evasión por Kalman (Capa 2):** Si la estimación del filtro cruza el umbral de seguridad, el robot gira. La dirección de giro se decide evaluando qué sensor lateral tiene mayor espacio libre.
4. **Deslizamiento (Capa 3):** Uso de sensores diagonales para realizar curvas suaves al rozar obstáculos sin detener el avance.
5. **Camino Libre (Capa 4):** Si la estimación es mayor al umbral de seguridad, el robot avanza en línea recta a velocidad crucero.

---

## 8. Experimentos realizados (Cruda vs Filtrada vs Kalman)
A continuación se presenta el comportamiento de las lecturas a lo largo del tiempo durante la navegación reactiva en ambos escenarios.

### **Escenario 1 - Fácil**
- La mayor parte del tiempo, la distancia se mantiene constante en el límite máximo de 0.06 metros (6 cm), formando una línea plana. Esto refleja que el robot tuvo mucho espacio libre ("Camino Libre").
- Se puede ver claramente los 4 "valles" principales (cerca de los 11s, 25s, 37s y 55s). Cada vez que la línea azul del Filtro de Kalman cruza hacia abajo la línea roja punteada (Umbral de Seguridad en 0.05m), el robot activa exitosamente su evasión.

**Gráfico del Escenario Simple**
<img width="1000" height="500" alt="escenario_facil" src="https://github.com/user-attachments/assets/4d95c0d3-2c63-47fb-b87d-5284dbe8112c" />

### **Escenario 2 - Complejo**
- A diferencia del primer escenario, aquí el robot pasa muy poco tiempo en "Camino Libre". Desde el segundo 3 hasta casi el final, las señales están en constante movimiento.
- Al fijarnos en el comportamiento del gráfico entre los 40 y los 60 segundos y luego entre los 70 y 90 segundos. Las caídas son constantes y profundas (llegando incluso a los 0.041m). La línea azul (Kalman) se muestra mucho más suave y decidida en sus picos comparada con la línea gris discontinua (Señal Cruda), lo que demuestra matemáticamente por qué el robot no se quedó "temblando" en los rincones del laberinto.

**Gráfico del Escenario Complejo**
<img width="1000" height="500" alt="escenario_complejo" src="https://github.com/user-attachments/assets/9bf3786b-5955-4eff-9858-10b27028ae93" />

---

## 9. Resultados en Escenarios de Prueba
Se diseñaron dos entornos en Webots para evaluar el rendimiento cuantitativo del controlador:
1. **Escenario Simple:** Entorno de ajedrez con tres obstáculos cúbicos de madera. 
   * **Tiempo de ejecución:** 60 segundos.
   * **Maniobras de evasión:** 9 maniobras.
   * **Colisiones registradas:** 0.
   * *Observación:* El robot logró evadirlos fluidamente, estabilizando su línea de marcha rápidamente tras cada maniobra sin dudar.
2. **Escenario Complejo:** Laberinto estrecho construido con cajas y elementos curvos. 
   * **Tiempo de ejecución:** 118 segundos.
   * **Maniobras de evasión:** 22 maniobras.
   * **Colisiones registradas:** 0.
   * *Observación:* La implementación de la "Capa de Deslizamiento" y la "Memoria de Giro" demostraron ser vitales, permitiendo al robot sortear pasillos ciegos y vueltas de 90° continuas (llegando hasta 4.1 cm de distancia) sin atascarse en mínimos locales.

---

## 10. Conclusiones
El sistema demostró ser robusto en ambos entornos, alcanzando 0 colisiones en los dos escenarios a pesar de las diferencias de complejidad entre ellos. La arquitectura de control por capas fue determinante para el buen desempeño. La capa de Kalman actúa de forma anticipada cuando la distancia estimada baja de 5 cm, mientras que la capa de pánico funciona como red de seguridad ante acercamientos bruscos que el filtro podría suavizar en exceso. La combinación de ambas cubrió todos los casos registrados sin necesidad de colisionar. Respecto al filtrado, el filtro exponencial (α = 0.3) introduce un retardo perceptible ante cambios rápidos, lo que en principio podría retrasar una decisión de evasión. Sin embargo, dado que la toma de decisiones se basa en la estimación de Kalman y no en la señal filtrada directamente, este retardo no impactó negativamente en la seguridad del robot. La estimación de Kalman resultó más reactiva que el filtro exponencial y más estable que la señal cruda, comportándose como el punto intermedio más adecuado para la toma de decisiones en tiempo real. El escenario complejo mostró mayor cantidad de evasiones (~11.2/min vs ~9.4/min) y distancias mínimas más bajas (4.1 cm vs 4.8 cm), evidenciando que el entorno exige más al sistema. Aun así, el rendimiento en seguridad fue idéntico, lo que sugiere que la lógica implementada escala bien a entornos más desafiantes. 
Finalmente, la fusión sensorial mediante el filtro de Kalman validó su utilidad frente al uso exclusivo de señales crudas o filtradas: al combinar la predicción de movimiento con la percepción del entorno, el robot logra anticipar obstáculos de forma más confiable, reduciendo la dependencia de reacciones de último momento.

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
