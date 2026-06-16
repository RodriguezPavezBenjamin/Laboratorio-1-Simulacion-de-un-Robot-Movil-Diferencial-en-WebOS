# Proyecto Final: Navegacion Autonoma con Planificacion de Rutas en Webots

## Integrantes

- Ignacio Layana
- Sebastian Rojas
- Javier Morales
- Benjamin Rodriguez

## Linea Seleccionada

Linea A: planificacion de rutas sobre grilla de ocupacion usando A*.

## Objetivo Del Proyecto

Diseñar e implementar en Webots un sistema de navegacion autonoma para un robot movil diferencial tipo e-puck, capaz de desplazarse desde una posicion inicial hasta una meta dentro de un entorno con obstaculos. La solucion integra control diferencial, uso de sensores de proximidad, encoders, odometria y una capa de navegacion global basada en A*.

## Relacion Con Los Laboratorios 1 y 2

### Laboratorio 1

- Se reutilizo el modelo de movimiento diferencial mediante velocidades independientes para las ruedas izquierda y derecha.
- Se implemento seguimiento de puntos intermedios de la ruta usando error angular y control de velocidades.
- Se trabajo con trayectorias rectas y cambios de orientacion para alcanzar waypoints consecutivos.

### Laboratorio 2

- Se integraron sensores de distancia `ps0` a `ps7` para detectar obstaculos cercanos.
- Se usaron los encoders de rueda (`left wheel sensor` y `right wheel sensor`) para estimar desplazamiento y orientacion por odometria.
- Se implemento una capa de evitacion reactiva local para corregir la trayectoria frente a riesgo de colision.

## Robot, Sensores y Actuadores

El robot utilizado es un e-puck definido dentro del mundo de Webots.

### Actuadores

- `left wheel motor`
- `right wheel motor`

### Sensores

- `8` sensores de distancia de proximidad
- `GPS` usado como referencia para comparar la posicion estimada
- `Compass` usado para observar la orientacion global del robot
- `PositionSensor` en ambas ruedas para odometria

## Descripcion General De La Solucion

La arquitectura del controlador tiene dos capas:

1. Navegacion global:
   Se representa el entorno mediante una grilla discreta `7x10`. Sobre esa grilla se calcula una ruta con A* desde la celda inicial hasta la meta usando conectividad de 4 vecinos y heuristica Manhattan.

2. Navegacion local:
   La ruta planificada se convierte a waypoints fisicos en el mundo. El robot sigue esos waypoints con control diferencial. Si detecta obstaculos cercanos con sus sensores de distancia, aplica una maniobra reactiva para evitar colision.

Ademas, el sistema estima la pose del robot por odometria y registra metricas de desempeno al terminar cada corrida.

## Representacion Del Entorno

Se definieron dos escenarios:

- `simple`: mundo base `worlds/ProyectoFinal.wbt`
- `complex`: mundo alternativo `worlds/ProyectoFinalComplejo.wbt`

Cada mundo usa el mismo controlador `controllers/delantero_controller/delantero_controller.c`, pero cambia la configuracion del escenario mediante `customData` del robot:

- `customData "simple"`
- `customData "complex"`

La grilla es de `7` filas por `10` columnas. Cada celda con valor `1` representa una zona bloqueada por obstaculos.

## Algoritmo Implementado

### Planificacion A*

El algoritmo A* trabaja sobre la grilla discreta y genera una secuencia ordenada de celdas desde inicio hasta meta.

- Vecindad: 4 vecinos
- Costo por paso: `1`
- Heuristica: distancia Manhattan

La salida del algoritmo es una lista de celdas que luego se convierten al centro fisico de cada celda usando:

- `CELL_SIZE`
- `GRID_ORIGIN_X`
- `GRID_ORIGIN_Y`

### Odometria

La pose del robot se actualiza con encoders de rueda usando:

- desplazamiento de rueda izquierda y derecha
- desplazamiento lineal medio
- cambio de orientacion por diferencia entre ruedas

Esta pose estimada se usa para el seguimiento de waypoints y se compara contra el `GPS` como referencia.

### Seguimiento De Ruta

Para cada waypoint:

- se calcula el error angular respecto a la pose estimada
- se ajustan las velocidades izquierda y derecha
- se reduce la velocidad al acercarse al punto objetivo

### Evitacion Reactiva

Si los sensores detectan proximidad por delante o por los laterales:

- el robot prioriza una maniobra de giro
- corrige lateralmente si el obstaculo esta solo a un costado

## Pseudocodigo De La Solucion

```text
inicializar robot, motores, GPS, compass, sensores de distancia y encoders
seleccionar escenario usando customData
cargar grilla, inicio y meta
calcular ruta A* sobre la grilla
convertir ruta a waypoints fisicos

mientras la simulacion este activa:
  leer sensores de proximidad
  leer encoders
  actualizar odometria
  leer GPS y compass como referencia

  si ya se alcanzaron todos los waypoints:
    detener robot
    imprimir metricas finales
    continuar

  obtener waypoint actual
  si waypoint actual fue alcanzado:
    avanzar al siguiente waypoint

  si existe riesgo de colision local:
    ejecutar maniobra reactiva
  si no:
    seguir waypoint con control diferencial

  registrar estado y metricas
```

## Escenarios De Prueba

### Escenario Simple

Archivo:

- `worlds/ProyectoFinal.wbt`

Caracteristicas:

- ruta mas directa
- menor numero de obstaculos relevantes
- menor cantidad de cambios de direccion

Objetivo:

- validar el funcionamiento base del planificador y del seguimiento de trayectoria

### Escenario Complejo

Archivo:

- `worlds/ProyectoFinalComplejo.wbt`

Caracteristicas:

- ruta mas larga
- mas celdas bloqueadas en la grilla
- mayor cantidad de cambios de direccion

Objetivo:

- evaluar el aumento del error odometrico y el costo de navegacion en una trayectoria mas exigente

## Resultados Experimentales

### Resumen De Metricas

| Metrica | Escenario Simple | Escenario Complex |
|---|---:|---:|
| Estado final | exito | exito |
| Tiempo simulado (s) | 111.17 | 203.78 |
| Ruta planificada (celdas) | 9 | 16 |
| Longitud ruta planificada (m) | 7.20 | 13.50 |
| Longitud trayectoria ejecutada (m) | 7.08 | 12.57 |
| Eventos de evasion total | 0 | 0 |
| Eventos de evasion frontal | 0 | 0 |
| Eventos de evasion izquierda | 0 | 0 |
| Eventos de evasion derecha | 0 | 0 |
| Giros significativos | 0 | 65 |
| Replanificaciones | 0 | 0 |
| Error de posicion final (m) | 0.008 | 0.569 |
| Error de posicion maximo (m) | 0.008 | 0.569 |

### Analisis De Resultados

- En el escenario simple el robot alcanzo la meta con una trayectoria casi recta y un error de posicion muy bajo.
- La diferencia entre la longitud planificada y la ejecutada fue pequena, lo que indica seguimiento estable de la ruta.
- En el escenario complejo el robot tambien alcanzo la meta, pero con una trayectoria mas larga y muchos mas cambios de direccion.
- El error final aumento de `0.008 m` a `0.569 m`, lo que evidencia acumulacion de error odometrico cuando la navegacion exige mas giros.
- No hubo eventos de evasion reactiva en estas pruebas, porque los mundos fueron alineados con la grilla para validar principalmente la navegacion global y el seguimiento.

## Instrucciones Para Ejecutar La Simulacion

### Escenario Simple

1. Abrir `worlds/ProyectoFinal.wbt` en Webots.
2. Abrir `controllers/delantero_controller/delantero_controller.c`.
3. Ejecutar `Build > Build`.
4. Ejecutar `Simulation > Reset`.
5. Ejecutar `Play`.

### Escenario Complejo

1. Abrir `worlds/ProyectoFinalComplejo.wbt` en Webots.
2. Abrir `controllers/delantero_controller/delantero_controller.c`.
3. Ejecutar `Build > Build`.
4. Ejecutar `Simulation > Reset`.
5. Ejecutar `Play`.

## Archivos Principales Del Proyecto

- `controllers/delantero_controller/delantero_controller.c`
- `worlds/ProyectoFinal.wbt`
- `worlds/ProyectoFinalComplejo.wbt`
- `protos/E-puck.proto`

## Evidencias

- Agregar capturas del escenario simple.
- Agregar capturas del escenario complejo.
- Agregar enlace a video demostrativo del proyecto.

## Limitaciones

- La planificacion se realiza sobre una grilla fija conocida a priori.
- No hay replanificacion dinamica efectiva frente a cambios del entorno durante la ejecucion.
- El sistema usa GPS solo como referencia de comparacion, no como parte del lazo principal de control.
- El error odometrico crece en trayectorias mas complejas.

## Mejoras Futuras

- Implementar replanificacion online al detectar bloqueos imprevistos.
- Incorporar filtrado simple adicional o fusion sensorial para reducir error acumulado.
- Registrar metricas en archivo CSV para facilitar analisis externo.
- Agregar visualizacion de la ruta sobre el mundo o exportar la trayectoria ejecutada.
- Explorar una version con SLAM o grilla de ocupacion actualizada en tiempo real.

## Conclusiones

Se logro implementar un sistema de navegacion autonoma para un robot diferencial en Webots usando planificacion A* sobre grilla, seguimiento de waypoints y estimacion por odometria. La solucion resolvio exitosamente dos escenarios de complejidad distinta. El escenario simple mostro alta precision y estabilidad, mientras que el escenario complejo evidencio el impacto de los giros y de la acumulacion de error odometrico. En conjunto, el proyecto cumple la idea central de integrar control diferencial, percepcion, estimacion y toma de decisiones global.
