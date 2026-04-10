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
FORWARD_SPEED = 0.15  # m/s (Velocidad de avance tangencial)
THRESHOLD_FRONT = 0.3  # m (Distancia para detectar el objeto al inicio)
THRESHOLD_SIDE = 0.3  # m (Distancia ideal/radio de la órbita)
KP_YAW = 25  # Ganancia: a mayor valor, giros más agresivos
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
                while True:
                    # Leemos la distancia actual al objeto (derecha)
                    dist_right = safe_read(mr.right)

                    # 1. Calculamos el error de distancia
                    # error > 0: estamos muy cerca
                    # error < 0: estamos muy lejos
                    error_side = THRESHOLD_SIDE - dist_right

                    # 2. Calculamos la velocidad de rotación (Yaw Rate)
                    # Usamos la fórmula: Yaw = Error * Ganancia
                    # Si estamos cerca, el yaw será positivo (giro izquierda) para alejarnos.
                    # Si estamos lejos, el yaw será negativo (giro derecha) para cerrarnos.
                    yaw_rate = error_side * KP_YAW

                    # Limitamos la rotación para mantener la estabilidad (máx 80 grad/s)
                    yaw_rate = np.clip(yaw_rate, -45, 45)

                    # 3. Ejecutamos el movimiento combinado
                    # Avanzamos (X) mientras rotamos sobre nuestro eje (Yaw)
                    # Esto genera una trayectoria curva perfecta
                    mc.start_linear_motion(FORWARD_SPEED, 0.0, 0.0, rate_yaw=yaw_rate)

                    # Pequeña seguridad frontal: si el objeto se mueve hacia nosotros, frenar
                    if safe_read(mr.front) < 0.3:
                        mc.stop()
                        print("¡Alerta frontal! Frenando...")
                        time.sleep(0.5)

                    time.sleep(LOOP_PERIOD)

            except KeyboardInterrupt:
                print("\nInterrupción detectada. Aterrizando de forma segura...")
                mc.stop()
                mc.land()

    print("=== MISIÓN FINALIZADA ===")
