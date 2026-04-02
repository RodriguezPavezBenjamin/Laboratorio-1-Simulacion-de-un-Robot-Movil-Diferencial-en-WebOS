# Laboratorio 1: Simulación de un Robot Móvil Diferencial en Webots

**Curso:** Robótica y Sistemas Autónomos 2026-01 — ICI 4150  
**Robot:** E-puck (diferencial de dos ruedas)  
**Simulador:** Webots R2025a

---

## Descripción del laboratorio

En este laboratorio se simula el comportamiento cinemático de un robot móvil diferencial (E-puck) en el entorno Webots. El objetivo es comprender cómo las velocidades de las ruedas izquierda y derecha determinan el movimiento del robot, aplicando el modelo cinemático diferencial:

$$v = \frac{v_r + v_l}{2} \qquad \omega = \frac{v_r - v_l}{L}$$

Donde `vr` es la velocidad de la rueda derecha, `vl` la de la rueda izquierda, y `L` la distancia entre ruedas.

---

## Estructura del repositorio

```
├── worlds/
│   └── Laboratorio1.wbt       # Mundo de simulación en Webots
├── controllers/
│   └── epuck_go_forward/
│       └── epuck_go_forward.py  # Controlador del robot (Python)
├── screenshots/               # Capturas de pantalla de los experimentos
└── README.md
```

---

## Cómo ejecutar la simulación en Webots

### Requisitos previos

- [Webots R2025a](https://cyberbotics.com/) instalado
- Python 3.x

### Pasos

1. Clonar este repositorio:
   ```bash
   git clone <URL_DEL_REPOSITORIO>
   cd <NOMBRE_DEL_REPOSITORIO>
   ```

2. Abrir Webots y cargar el mundo:
   - Ir a **File → Open World**
   - Seleccionar el archivo `worlds/Laboratorio1.wbt`

3. Ejecutar la simulación:
   - Presionar el botón **Play ▶** en Webots
   - El robot comenzará a moverse según el controlador activo

4. Para cambiar el experimento, modificar las velocidades en `controllers/epuck_go_forward/epuck_go_forward.py`:
   ```python
   left_motor.setVelocity(vl)
   right_motor.setVelocity(vr)
   ```

---

## Experimentos realizados

| Experimento | `vl` | `vr` | Trayectoria esperada |
|---|---|---|---|
| Movimiento recto | `v` | `v` | Línea recta |
| Curva a la derecha | `v` | `v/2` | Arco curvo |
| Rotación en el lugar | `v` | `-v` | Giro sobre su eje |
| Círculo | `v` | `v * r` | Trayectoria circular |

---

## Resultados obtenidos

**Movimiento recto:** Al asignar la misma velocidad a ambas ruedas, el robot avanzó en línea recta sin desviarse.

**Trayectoria curva:** Con velocidades distintas, el robot describió un arco, curvándose hacia el lado de la rueda más lenta.

**Rotación en el lugar:** Con velocidades iguales pero de signo opuesto, el robot giró sobre su propio eje.

**Círculo:** Ajustando la relación entre `vl` y `vr` de forma proporcional, el robot trazó una trayectoria circular.

---

## Preguntas de análisis

**1. ¿Qué ocurre cuando ambas ruedas tienen la misma velocidad?**  
El robot se desplaza en línea recta. La velocidad angular ω es cero, por lo que no hay rotación.

**2. ¿Cómo cambia la trayectoria cuando las velocidades son diferentes?**  
El robot describe una curva. Cuanto mayor sea la diferencia entre `vr` y `vl`, más cerrada será la curva. El robot gira hacia el lado de la rueda más lenta.

**3. ¿Qué ocurre cuando una rueda gira en sentido opuesto a la otra?**  
El robot rota sobre su propio eje (en el lugar). La velocidad lineal v es cero y la velocidad angular ω es máxima.

**4. ¿Qué tipo de movimiento permite dibujar un círculo?**  
Se necesita una diferencia constante entre `vr` y `vl`, manteniendo ambas del mismo signo. Esto genera una velocidad angular constante y una trayectoria circular de radio `r = (L/2) * (vr + vl) / (vr - vl)`.

---

## Integrantes

| Rol | Nombre |
|---|---|
| Programador | Sebastian Rojas |
| Experimentador | Ignacio Layana |
| Analista | Benjamin Rodriguez |
| Documentador | Benjamin Rodriguez |
| Integrador |Javier Morales |