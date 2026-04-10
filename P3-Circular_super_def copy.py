#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Crazyflie 2.1: Órbita Circular Suave (Cilíndrica)
El dron busca un objeto, se posiciona de lado y lo rodea manteniendo
una distancia constante mediante el ajuste de su velocidad de rotación (yaw).
Ideal para rodear personas o columnas.
"""

import logging
import time
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.multiranger import Multiranger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

# --- Configuración de Conexión ---
URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E701")

# --- Parámetros de Comportamiento ---
FORWARD_SPEED = 0.15  # m/s (Velocidad de avance tangencial)
THRESHOLD_FRONT = 0.3  # m (Distancia para detectar el objeto al inicio)
THRESHOLD_SIDE = 0.3  # m (Distancia ideal/radio de la órbita)
KP_YAW = 25.0  # Ganancia: a mayor valor, giros más agresivos
LOOP_PERIOD = 0.1  # s (Frecuencia de actualización de 10Hz)

# Parámetros para robustez en órbita
SIDE_DEADBAND = 0.05  # m (No corregir dentro de este margen)
MAX_YAW_RATE = 55.0  # deg/s (Límite de giro en modo órbita)

# Parámetros de recuperación cuando se pierde el lateral
SIDE_LOST_DISTANCE = 1.2  # m (Si supera esto, se considera pérdida lateral)
LOST_CYCLES_TO_RECOVER = 3  # ciclos consecutivos para entrar en recuperación
RECOVERY_YAW_RATE = 24.0  # deg/s (Giro constante de búsqueda)
RECOVERY_REORIENT_YAW_RATE = 32.0  # deg/s (Giro cuando frontal detecta cerca)
RECOVERY_FORWARD_SPEED = 0.04  # m/s (Avance suave durante búsqueda)
FRONT_REORIENT_THRESHOLD = 0.45  # m (Ayuda frontal para recolocar)
FRONT_CONFIRM_CYCLES = 2  # ciclos para confirmar frontal cercano
REVERSE_REORIENT_CYCLES = 6  # ciclos de giro contrario para recolocar
REACQUIRE_BAND = 0.12  # m (Banda de distancia válida para reenganche)
REACQUIRE_STABLE_CYCLES = 2  # ciclos consecutivos para volver a órbita
FRONT_HARD_STOP = 0.05  # m (Seguridad frontal fuerte)


def safe_read(dist):
    """Evita errores si el sensor no detecta nada devolviendo un valor lejano."""
    return dist if dist is not None else 4.0


if __name__ == "__main__":
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        # Despegue automático a 0.4 metros
        with MotionCommander(scf, default_height=0.4) as mc, Multiranger(scf) as mr:
            print("=== INICIANDO ÓRBITA CIRCULAR ===")
            time.sleep(1.0)

            # --- FASE 1: APROXIMACIÓN ---
            print("Buscando objeto al frente...")
            while safe_read(mr.front) > THRESHOLD_FRONT:
                mc.start_linear_motion(0.1, 0.0, 0.0)
                time.sleep(LOOP_PERIOD)

            mc.stop()
            print("Objeto detectado. Posicionando costado derecho...")

            # --- FASE 2: GIRO INICIAL ---
            # Giramos 90 grados para que el objeto quede a nuestra derecha
            mc.turn_left(90)
            time.sleep(1.0)

            # --- FASE 3: BUCLE DE ÓRBITA CONTINUA ---
            print("Orbitando... (Presiona Ctrl+C para aterrizar)")
            try:
                mode = "ORBIT"
                lost_counter = 0
                reacquire_counter = 0
                front_near_counter = 0
                reverse_reorient_counter = 0

                while True:
                    dist_front = safe_read(mr.front)
                    raw_right = mr.right
                    dist_right = safe_read(raw_right)
                    side_visible = raw_right is not None and dist_right < SIDE_LOST_DISTANCE

                    if mode == "ORBIT":
                        if side_visible:
                            lost_counter = 0

                            # Control proporcional con banda muerta para evitar oscilaciones por ruido.
                            error_side = THRESHOLD_SIDE - dist_right
                            if abs(error_side) < SIDE_DEADBAND:
                                error_side = 0.0

                            yaw_rate = error_side * KP_YAW
                            yaw_rate = np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE)

                            mc.start_linear_motion(FORWARD_SPEED, 0.0, 0.0, rate_yaw=yaw_rate)
                        else:
                            lost_counter += 1
                            mc.start_linear_motion(
                                RECOVERY_FORWARD_SPEED,
                                0.0,
                                0.0,
                                rate_yaw=RECOVERY_YAW_RATE,
                            )
                            if lost_counter >= LOST_CYCLES_TO_RECOVER:
                                mode = "RECOVERY"
                                reacquire_counter = 0
                                front_near_counter = 0
                                reverse_reorient_counter = 0
                                print("Referencia lateral perdida -> Modo RECOVERY")

                    else:  # mode == "RECOVERY"
                        # Por defecto se mantiene giro positivo de búsqueda.
                        # Si el frontal confirma cercanía, se aplica un tramo corto de
                        # giro contrario para recolocar tras posible sobrepaso.
                        if dist_front < FRONT_REORIENT_THRESHOLD:
                            front_near_counter += 1
                        else:
                            front_near_counter = 0

                        if front_near_counter >= FRONT_CONFIRM_CYCLES:
                            reverse_reorient_counter = REVERSE_REORIENT_CYCLES
                            front_near_counter = 0

                        if reverse_reorient_counter > 0:
                            cmd_forward = 0.0
                            cmd_yaw = -RECOVERY_REORIENT_YAW_RATE
                            reverse_reorient_counter -= 1
                        else:
                            cmd_forward = RECOVERY_FORWARD_SPEED
                            cmd_yaw = RECOVERY_YAW_RATE

                        mc.start_linear_motion(cmd_forward, 0.0, 0.0, rate_yaw=cmd_yaw)

                        if side_visible and (abs(dist_right - THRESHOLD_SIDE) <= REACQUIRE_BAND):
                            reacquire_counter += 1
                        else:
                            reacquire_counter = 0

                        if reacquire_counter >= REACQUIRE_STABLE_CYCLES:
                            mode = "ORBIT"
                            lost_counter = 0
                            print("Referencia lateral recuperada -> Volviendo a ORBIT")

                    # Seguridad frontal global
                    if dist_front < FRONT_HARD_STOP:
                        mc.stop()
                        print("¡Alerta frontal! Frenando...")
                        time.sleep(0.3)

                    time.sleep(LOOP_PERIOD)

            except KeyboardInterrupt:
                print("\nInterrupción detectada. Aterrizando de forma segura...")
                mc.stop()
                mc.land()

    print("=== MISIÓN FINALIZADA ===")
