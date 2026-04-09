#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Wall-following simplificado para Crazyflie 2.1 + Flow Deck + Multi-Ranger Deck.
Se busca el muro, se aproxima hasta umbral, y luego se sigue lateralmente
manteniendo la distancia deseada. Si el frente detecta demasiado cerca,
retrocede y gira para evitar colisión.
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

# URI de conexión (ajusta si hace falta)
URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E702")

# Par ámetros de comportamiento
SEARCH_SPEED = 0.1  # m/s al buscar el muro
FOLLOW_SPEED = 0.08  # m/s al avanzar junto al muro
BACK_SPEED = 0.15  # m/s al retroceder en evasión
TURN_ANGLE = 30  # grados a girar en evasión
LOOP_PERIOD = 0.1  # s entre iteraciones (10 Hz)
THRESHOLD_FRONT = 0.5  # m, distancia mínima al frente
THRESHOLD_SIDE = 0.3  # m, distancia objetivo al muro
KP_SIDE = 0.5  # ganancia proporcional para corrección lateral


def safe_read(dist):
    """Devuelve la distancia o un valor grande si es None."""
    return dist if dist is not None else 5.0


if __name__ == "__main__":
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    print(f"Intentando conectar a {URI}...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        print("Conectado con éxito!")
        with MotionCommander(scf, default_height=0.4) as mc, Multiranger(scf) as mr:
            print("Despegando... buscando muro")
            time.sleep(1.0)

            # 1) Fase de búsqueda: avanzar hasta detectar muro
            while True:
                front = safe_read(mr.front)
                if front < THRESHOLD_FRONT * 2:  # empieza a estar cerca
                    print(f"Muro detectado a ~{front:.2f} m, paso a SEGUIR")
                    mc.stop()
                    break
                mc.start_linear_motion(SEARCH_SPEED, 0.0, 0.0)
                time.sleep(LOOP_PERIOD)

            # 2) Fase de seguimiento de pared
            try:
                while True:
                    front = safe_read(mr.front)
                    side = safe_read(mr.right)  # si sigues a la derecha
                    # (usa mr.left si sigues muro izquierdo)

                    # Evasión urgente si el frente está muy cerca
                    if front < THRESHOLD_FRONT:
                        print(f"CHOCARÍA ({front:.2f} m) → retroceso y giro")
                        mc.stop()
                        mc.back(0.1, velocity=BACK_SPEED)
                        mc.turn_left(TURN_ANGLE)
                        time.sleep(0.5)  # dejar girar
                        continue

                    # Control proporcional lateral: vy >0 = mover dron a su derecha
                    error_side = THRESHOLD_SIDE - side
                    vy = np.clip(error_side * KP_SIDE, -0.2, 0.2)

                    # Avanza + corrige lateralmente
                    mc.start_linear_motion(FOLLOW_SPEED, vy, 0.0)
                    time.sleep(LOOP_PERIOD)

            except KeyboardInterrupt:
                print("Interrupción: aterrizando…")
                mc.stop()

    print("Demo finalizada.")
