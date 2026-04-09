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
URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E702")

# --- Parámetros de Comportamiento ---
FORWARD_SPEED = 0.12  # m/s (Reducido para mayor estabilidad y precisión)
THRESHOLD_FRONT = 0.5  # m (Distancia para detectar el objeto y entrar en órbita)
THRESHOLD_SIDE = 0.4  # m (Radio de la órbita: 0.4m recomendado)
KP_YAW = 25.0  # Ganancia proporcional para corrección
LOOP_PERIOD = 0.1  # s (Frecuencia de actualización de 10Hz)


def safe_read(dist):
    """Evita errores si el sensor no detecta nada devolviendo un valor lejano."""
    return dist if dist is not None else 4.0


if __name__ == "__main__":
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        # Despegue automático a 0.4 metros
        with MotionCommander(scf, default_height=0.4) as mc, Multiranger(scf) as mr:
            print("=== INICIANDO ÓRBITA CIRCULAR PERFECCIONADA ===")
            time.sleep(1.0)

            # --- FASE 1: APROXIMACIÓN ---
            print(f"Buscando objeto al frente (umbral {THRESHOLD_FRONT}m)...")
            while safe_read(mr.front) > THRESHOLD_FRONT:
                mc.start_linear_motion(0.1, 0.0, 0.0)
                time.sleep(LOOP_PERIOD)

            mc.stop()
            print("Objeto detectado. Girando para posicionamiento lateral...")
            time.sleep(0.5)

            # --- FASE 2: GIRO INICIAL ---
            # Giramos 90 grados para que el objeto quede a nuestra derecha
            mc.turn_left(90)
            time.sleep(1.0)

            # --- FASE 3: BUCLE DE ÓRBITA CONTINUA ---
            print("Orbitando... (Presiona Ctrl+C para aterrizar)")

            # Calculamos el giro constante teórico (Yaw Nominal) necesario para el círculo
            # Yaw (rad/s) = Velocidad / Radio -> convertimos a grados/s
            # El signo es NEGATIVO porque el objeto está a la DERECHA y giramos a la DERECHA (-)
            base_yaw = -(FORWARD_SPEED / THRESHOLD_SIDE) * (180.0 / np.pi)

            try:
                while True:
                    # Leemos la distancia actual al objeto (derecha)
                    dist_right = safe_read(mr.right)

                    # 1. Calculamos el error de distancia
                    error_side = THRESHOLD_SIDE - dist_right

                    # 2. Velocidad de rotación final = Giro Base + Ajuste PID
                    # ajuste > 0 (giro izq) si estamos muy cerca
                    # ajuste < 0 (giro der) si estamos muy lejos
                    adjustment = error_side * KP_YAW
                    yaw_rate = base_yaw + adjustment

                    # Limitamos la rotación para mantener la estabilidad
                    yaw_rate = np.clip(yaw_rate, -85, 85)

                    # 3. Ejecutamos el movimiento combinado
                    mc.start_linear_motion(FORWARD_SPEED, 0.0, 0.0, rate_yaw=yaw_rate)

                    # Seguridad frontal: si algo bloquea el camino de la órbita
                    if safe_read(mr.front) < 0.3:
                        mc.stop()
                        print("¡Obstrucción frontal detectada! Pausando...")
                        time.sleep(1.0)
                        # No salimos, reintentamos en la siguiente iteración

                    time.sleep(LOOP_PERIOD)

            except KeyboardInterrupt:
                print("\nFinalizando misión por usuario. Aterrizando...")
                mc.stop()
                mc.land()

    print("=== MISIÓN FINALIZADA ===")
