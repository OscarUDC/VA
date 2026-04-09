#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Crazyflie 2.1 + Multi-Ranger: Superación de obstáculos (Escalador)
El dron avanza, sube al detectar un obstáculo, lo cruza y aterriza
solo cuando confirma suelo firme mediante un filtro de confianza.
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

# --- Configuración ---
URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E702")

# Velocidades
SPEED_FORWARD = 0.2  # m/s
SPEED_CLIMB = 0.30  # m/s
SPEED_DESCEND = 0.10  # m/s

# Umbrales y Seguridad
THRESHOLD_FRONT = 0.8  # m, distancia para empezar a subir
THRESHOLD_GROUND = 0.8  # m, distancia que se considera "suelo real"
CONFIDENCE_REQUIRED = 15  # lecturas seguidas (1.5s a 10Hz)
LOOP_PERIOD = 0.1  # s (10 Hz)


def safe_read(dist):
    """Devuelve la distancia o un valor grande si es None."""
    return dist if dist is not None else 5.0


if __name__ == "__main__":
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        # Despegamos a una altura inicial de 0.4m
        with MotionCommander(scf, default_height=0.4) as mc, Multiranger(scf) as mr:
            print("=== INICIO DE MISIÓN: SUPERAR OBSTÁCULO ===")
            time.sleep(1.0)

            # --- ESTADO 1: AVANCE HASTA OBSTÁCULO ---
            print("Fase 1: Avanzando hacia el objetivo...")
            while safe_read(mr.front) > THRESHOLD_FRONT:
                mc.start_linear_motion(SPEED_FORWARD, 0.0, 0.0)
                time.sleep(LOOP_PERIOD)

            mc.stop()
            print(f"Obstáculo detectado a {safe_read(mr.front):.2f}m")

            # --- ESTADO 2: SUBIDA VERTICAL ---
            print("Fase 2: Subiendo...")
            # Subimos hasta que el sensor frontal ya no vea nada (esté despejado)
            while safe_read(mr.front) < 1.5:
                mc.start_linear_motion(0.0, 0.0, SPEED_CLIMB)
                time.sleep(LOOP_PERIOD)

            mc.stop()
            # Un pequeño extra de subida para asegurar que las patas no rocen
            mc.up(0.6)
            print("Altura suficiente alcanzada. Estabilizando...")
            mc.stop()
            time.sleep(1.0)

            # --- ESTADO 3: CRUCE Y BÚSQUEDA DE SUELO ---
            print("Fase 3: Cruce inicial para superar el borde...")

            print("Verificando suelo firme...")
            confidence_counter = 0

            while confidence_counter < CONFIDENCE_REQUIRED:
                # Seguimos avanzando mientras comprobamos abajo
                mc.forward(0.5, velocity=SPEED_FORWARD)

                dist_down = safe_read(mr.down)

                # Si el sensor de abajo lee una distancia grande, es que el objeto ya pasó
                if dist_down > THRESHOLD_GROUND:
                    confidence_counter += 1
                    print(dist_down)
                    print(
                        f" Confirmando suelo: {confidence_counter}/{CONFIDENCE_REQUIRED}"
                    )
                else:
                    # Si detectamos algo cerca, reiniciamos el contador (seguimos sobre el objeto)
                    if confidence_counter > 0:
                        print(
                            " Ruido o persona detectada abajo, reiniciando filtro.",
                            dist_down,
                        )
                    confidence_counter = 0

                time.sleep(LOOP_PERIOD)

            # --- ESTADO 4: FINALIZACIÓN ---
            mc.stop()
            print("Fase 4: Zona de aterrizaje confirmada. Margen de seguridad...")
            mc.move_distance(
                0.3, 0.0, 0.0
            )  # Avanzamos 30cm finales para alejarnos del borde

            print("Aterrizando...")
            mc.land()

    print("=== MISIÓN FINALIZADA CON ÉXITO ===")
