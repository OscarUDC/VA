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
FORWARD_SPEED   = 0.12  # m/s (Ligeramente más lento para mayor precisión)
THRESHOLD_FRONT = 0.6   # m (Distancia para detectar el objeto)
THRESHOLD_SIDE  = 0.5   # m (Radio de la órbita: 0.5m)
KP_YAW          = 25.0  # Ganancia corregida para evitar sobre-rotación
LOOP_PERIOD     = 0.1   # Actualización a 10Hz para mayor fluidez


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
                mc.start_linear_motion(FORWARD_SPEED, 0.0, 0.0)
                time.sleep(LOOP_PERIOD)

            mc.stop()
            print("Objeto detectado. Posicionando en lateral...")
            time.sleep(0.5)

            # --- FASE 2: ORIENTACIÓN ---
            # Giramos 90 grados para poner el objeto al costado derecho
            mc.turn_left(90)
            time.sleep(1.0)

            # --- FASE 3: BUCLE DE ÓRBITA ROBUSTA ---
            print("Orbitando con control de radio... (Ctrl+C para parar)")
            try:
                # Calculamos el giro constante teórico (Yaw Nominal)
                # Yaw = V / R (rad/s) -> convertimos a grados/s
                base_yaw = -(FORWARD_SPEED / THRESHOLD_SIDE) * (180.0 / np.pi)

                while True:
                    dist_right = safe_read(mr.right)

                    # Seguridad: Si el objeto se aleja demasiado, frenar para no orbitar al vacío
                    if dist_right > 1.2:
                        print("Fase 3: Objeto perdido. Deteniendo para re-enganchar...")
                        mc.stop()
                        time.sleep(0.5)
                        continue

                    # Control Proporcional (PD sencillo)
                    # error > 0: muy cerca -> giro izquierda (yaw +)
                    # error < 0: muy lejos  -> giro derecha (yaw -)
                    error_side = THRESHOLD_SIDE - dist_right
                    adjustment = error_side * KP_YAW

                    # Combinación: Giro base + Ajuste de corrección
                    yaw_rate = base_yaw + adjustment
                    
                    # Limitar rotación para máxima estabilidad
                    yaw_rate = np.clip(yaw_rate, -70, 70)

                    # Movimiento combinado: Avance X + Rotación Z
                    mc.start_linear_motion(FORWARD_SPEED, 0.0, 0.0, rate_yaw=yaw_rate)

                    # Detección de colisión frontal (obstáculo inesperado en la ruta)
                    if safe_read(mr.front) < 0.3:
                        mc.stop()
                        print("!Alerta frontal detectada, frenando!")
                        time.sleep(0.5)

                    time.sleep(LOOP_PERIOD)

            except KeyboardInterrupt:
                print("\nInterrupción detectada. Aterrizando de forma segura...")
                mc.stop()
                mc.land()

    print("=== MISIÓN FINALIZADA ===")
