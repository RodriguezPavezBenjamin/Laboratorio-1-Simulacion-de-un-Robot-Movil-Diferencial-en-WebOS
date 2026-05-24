import math
from controller import Robot, DistanceSensor, Motor

# ==========================================
# 1. PARÁMETROS Y CONFIGURACIÓN
# ==========================================
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())  
tiempo_muestreo_segundos = TIME_STEP / 1000.0                    

RADIO_RUEDA_METROS = 0.0205   
VELOCIDAD_MAX_RAD_SEG = 6.28        

UMBRAL_SEGURIDAD_METROS = 0.05  
UMBRAL_DIAGONALES_METROS = 0.055 
UMBRAL_PANICO_METROS = 0.035 

# ==========================================
# 2. INICIALIZACIÓN
# ==========================================
motor_izquierdo = robot.getDevice('left wheel motor')
motor_derecho = robot.getDevice('right wheel motor')
motor_izquierdo.setPosition(float('inf'))
motor_derecho.setPosition(float('inf'))
motor_izquierdo.setVelocity(0.0)
motor_derecho.setVelocity(0.0)

# Habilitar sensores frontales, laterales y diagonales
nombres_sensores = ['ps0', 'ps1', 'ps2', 'ps5', 'ps6', 'ps7']
diccionario_sensores = {}
for nombre in nombres_sensores:
    diccionario_sensores[nombre] = robot.getDevice(nombre)
    diccionario_sensores[nombre].enable(TIME_STEP) 

encoder_izquierdo = robot.getDevice('left wheel sensor')
encoder_derecho = robot.getDevice('right wheel sensor')
encoder_izquierdo.enable(TIME_STEP)
encoder_derecho.enable(TIME_STEP)

# ==========================================
# 3. VARIABLES DE ESTADO Y FILTRADO
# ==========================================
lectura_anterior_encoder_izq = 0.0
lectura_anterior_encoder_der = 0.0
es_primer_paso_simulacion = True  

factor_suavizado_alpha = 0.3
medicion_frontal_filtrada_anterior = 0.0  

distancia_estimada_kalman = 0.06      
covarianza_error_estimacion = 1.0           
ruido_proceso_encoders = 0.005         
ruido_medicion_sensor = 0.002          

# Almacenamiento en memoria para el resumen final
historial_tiempo = []
historial_distancia_cruda = []
historial_distancia_filtrada_simple = []
historial_distancia_estimada_kalman = []

def prox_to_meters(val):
    if val < 80:
        return 0.06  
    elif val > 3500:
        return 0.01  
    else:
        distancia_calculada = -0.0132 * math.log(val) + 0.1178
        return max(0.01, min(distancia_calculada, 0.06))
        
pasos_giro_restantes = 0  
velocidad_motor_izquierdo = 0.0
velocidad_motor_derecho = 0.0

vel_evasion_izq_bloqueada = 0.0
vel_evasion_der_bloqueada = 0.0

# --- VARIABLES ANALÍTICAS PARA EL INFORME ---
contador_evasiones = 0
contador_choques = 0
en_colision = False  # Evita contar el mismo choque múltiples veces por segundo

# ==========================================
# 4. BUCLE PRINCIPAL
# ==========================================
paso = 0

while robot.step(TIME_STEP) != -1:
    tiempo_actual_segundos = paso * tiempo_muestreo_segundos  
    
    # --- LECTURA DE SENSORES ---
    lectura_analogica_sensor_frontal_der = diccionario_sensores['ps0'].getValue()
    lectura_analogica_sensor_frontal_izq = diccionario_sensores['ps7'].getValue()
    lectura_analogica_sensor_lateral_der = diccionario_sensores['ps2'].getValue()  
    lectura_analogica_sensor_lateral_izq = diccionario_sensores['ps5'].getValue()  
    lectura_analogica_sensor_diag_der = diccionario_sensores['ps1'].getValue()
    lectura_analogica_sensor_diag_izq = diccionario_sensores['ps6'].getValue()
    
    z_frontal_cruda = min(prox_to_meters(lectura_analogica_sensor_frontal_der), 
                          prox_to_meters(lectura_analogica_sensor_frontal_izq))
    dist_diag_der = prox_to_meters(lectura_analogica_sensor_diag_der)
    dist_diag_izq = prox_to_meters(lectura_analogica_sensor_diag_izq)
    
    # Lógica de detección de colisión (< 1.5 cm)
    if z_frontal_cruda <= 0.015:
        if not en_colision:
            contador_choques += 1
            en_colision = True
    else:
        en_colision = False
    
    # --- ENCODERS Y AVANCE ---
    left_enc = encoder_izquierdo.getValue()
    right_enc = encoder_derecho.getValue()
    
    if es_primer_paso_simulacion:
        lectura_anterior_encoder_izq = left_enc
        lectura_anterior_encoder_der = right_enc
        medicion_frontal_filtrada_anterior = z_frontal_cruda
        es_primer_paso_simulacion = False
        continue  
        
    delta_th_izq = left_enc - lectura_anterior_encoder_izq
    delta_th_der = right_enc - lectura_anterior_encoder_der
    lectura_anterior_encoder_izq = left_enc
    lectura_anterior_encoder_der = right_enc
    delta_robot = (RADIO_RUEDA_METROS * delta_th_izq + RADIO_RUEDA_METROS * delta_th_der) / 2.0

    # --- FILTRO SIMPLE ---
    z_frontal_filtrada = factor_suavizado_alpha * z_frontal_cruda + (1 - factor_suavizado_alpha) * medicion_frontal_filtrada_anterior
    medicion_frontal_filtrada_anterior = z_frontal_filtrada

    # --- FILTRO DE KALMAN ---
    d_pred = distancia_estimada_kalman - delta_robot
    P_pred = covarianza_error_estimacion + ruido_proceso_encoders  
    ganancia_kalman = P_pred / (P_pred + ruido_medicion_sensor)                                      
    distancia_estimada_kalman = d_pred + ganancia_kalman * (z_frontal_cruda - d_pred)  
    covarianza_error_estimacion = (1 - ganancia_kalman) * P_pred                                      

    # --- ALMACENAMIENTO DE DATOS EN MEMORIA ---
    historial_tiempo.append(tiempo_actual_segundos)
    historial_distancia_cruda.append(z_frontal_cruda)
    historial_distancia_filtrada_simple.append(z_frontal_filtrada)
    historial_distancia_estimada_kalman.append(distancia_estimada_kalman)

    # ==========================================
    # F. ARQUITECTURA DE CONTROL ROBUSTA
    # ==========================================
    
    # Bloqueo de decisión: Solo evalúa hacia dónde girar si no está evadiendo algo ya
    if pasos_giro_restantes == 0:
        if lectura_analogica_sensor_lateral_izq > lectura_analogica_sensor_lateral_der:
            vel_evasion_izq_bloqueada = VELOCIDAD_MAX_RAD_SEG * 0.5
            vel_evasion_der_bloqueada = -VELOCIDAD_MAX_RAD_SEG * 0.5
        else:
            vel_evasion_izq_bloqueada = -VELOCIDAD_MAX_RAD_SEG * 0.5
            vel_evasion_der_bloqueada = VELOCIDAD_MAX_RAD_SEG * 0.5

    # [CAPA 0]: Reflejo de Pánico (Evita choques inminentes ignorando el filtro)
    if z_frontal_cruda <= UMBRAL_PANICO_METROS and pasos_giro_restantes <= 5:
        if pasos_giro_restantes == 0:  
            contador_evasiones += 1
        pasos_giro_restantes = 25  
        velocidad_motor_izquierdo = vel_evasion_izq_bloqueada
        velocidad_motor_derecho = vel_evasion_der_bloqueada
        covarianza_error_estimacion = 1.0  

    # [CAPA 1]: Ejecución persistente de la maniobra (Memoria de giro)
    elif pasos_giro_restantes > 0:
        pasos_giro_restantes -= 1
        velocidad_motor_izquierdo = vel_evasion_izq_bloqueada
        velocidad_motor_derecho = vel_evasion_der_bloqueada
        covarianza_error_estimacion = 1.0  
        
    # [CAPA 2]: Evasión Frontal por Kalman (Inteligente y anticipada)
    elif distancia_estimada_kalman <= UMBRAL_SEGURIDAD_METROS:
        contador_evasiones += 1  
        pasos_giro_restantes = 18  
        velocidad_motor_izquierdo = vel_evasion_izq_bloqueada
        velocidad_motor_derecho = vel_evasion_der_bloqueada

    # [CAPA 3]: Deslizamiento por bordes (Diagonales)
    elif dist_diag_izq <= UMBRAL_DIAGONALES_METROS:
        velocidad_motor_izquierdo = VELOCIDAD_MAX_RAD_SEG * 0.4
        velocidad_motor_derecho = VELOCIDAD_MAX_RAD_SEG * 0.05
    elif dist_diag_der <= UMBRAL_DIAGONALES_METROS:
        velocidad_motor_izquierdo = VELOCIDAD_MAX_RAD_SEG * 0.05
        velocidad_motor_derecho = VELOCIDAD_MAX_RAD_SEG * 0.4

    # [CAPA 4]: Camino Libre
    else:
        velocidad_motor_izquierdo = VELOCIDAD_MAX_RAD_SEG * 0.4
        velocidad_motor_derecho = VELOCIDAD_MAX_RAD_SEG * 0.4

    # --- APLICAR MOTORES ---
    motor_izquierdo.setVelocity(velocidad_motor_izquierdo)
    motor_derecho.setVelocity(velocidad_motor_derecho)
    
    paso += 1
    
    # --- CONSOLA EN TIEMPO REAL ---
    if paso % 10 == 0:
        print(f"T: {tiempo_actual_segundos:.2f}s | Cruda: {z_frontal_cruda:.3f}m | "
              f"Filtrada: {z_frontal_filtrada:.3f}m | Kalman: {distancia_estimada_kalman:.3f}m")

    # Imprime el resumen analítico actualizado cada 50 pasos (aprox. cada 1.5 segundos)
    if paso % 100 == 0:
        print("\n" + "-"*45)
        print(f"--- REPORTE ACTUAL (T: {tiempo_actual_segundos:.2f}s) ---")
        print(f"Maniobras de evasión : {contador_evasiones}")
        print(f"Colisiones (< 1.5cm) : {contador_choques}")
        print("-"*45 + "\n")
