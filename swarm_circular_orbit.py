#!/usr/bin/env python3
"""
Crazyflie 2.1 - Circular orbit with Swarm
Runs the same ORBIT/RECOVERY behavior in parallel for multiple drones.
"""

import logging
import time
import numpy as np

import cflib.crtp
from cflib.crazyflie.swarm import Swarm, CachedCfFactory
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

# Replace with your real URIs (one per Crazyflie)
URI_LIST = [
    "radio://0/80/2M/E7E7E7E701",  # Crazyflie #1
    "radio://0/80/2M/E7E7E7E702",  # Crazyflie #2
]

# Parámetros de Comportamiento (exacto a P3-Circular_super_def.py)
DEFAULT_HEIGHT = 0.4
FORWARD_SPEED = 0.15  # m/s (Velocidad de avance tangencial)
THRESHOLD_FRONT = 0.3  # m (Distancia para detectar el objeto al inicio)
THRESHOLD_SIDE = 0.3   # m (Distancia ideal/radio de la órbita)
KP_YAW = 25            # Ganancia: a mayor valor, giros más agresivos
LOOP_PERIOD = 0.1      # s (Frecuencia de actualización de 10Hz)


def safe_read(dist):
    return dist if dist is not None else 4.0


def swarm_circular_orbit(scf):
    """Órbita circular suave (cilíndrica) para cada dron en el enjambre."""
    uri = scf.cf.link_uri
    print(f"[{uri}] === INICIANDO ÓRBITA CIRCULAR ===")

    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc, Multiranger(scf) as mr:
        time.sleep(1.0)

        # --- FASE 1: APROXIMACIÓN ---
        print(f"[{uri}] Buscando objeto al frente...")
        while safe_read(mr.front) > THRESHOLD_FRONT:
            mc.start_linear_motion(0.1, 0.0, 0.0)
            time.sleep(LOOP_PERIOD)

        mc.stop()
        print(f"[{uri}] Objeto detectado. Posicionando costado derecho...")

        # --- FASE 2: GIRO INICIAL ---
        # Giramos 90 grados para que el objeto quede a nuestra derecha
        mc.turn_left(90)
        time.sleep(1.0)

        # --- FASE 3: BUCLE DE ÓRBITA CONTINUA ---
        print(f"[{uri}] Orbitando... (Presiona Ctrl+C para aterrizar)")
        total_yaw = 0.0  # Grados acumulados para el giro
        
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

                # Limitamos la rotación para mantener la estabilidad (máx 45 grad/s)
                yaw_rate = np.clip(yaw_rate, -45, 45)

                # 3. Ejecutamos el movimiento combinado
                # Avanzamos (X) mientras rotamos sobre nuestro eje (Yaw)
                # Esto genera una trayectoria curva perfecta
                mc.start_linear_motion(FORWARD_SPEED, 0.0, 0.0, rate_yaw=yaw_rate)

                # Pequeña seguridad frontal: si el objeto se mueve hacia nosotros, frenar
                if safe_read(mr.front) < 0.2:
                    mc.stop()
                    print(f"[{uri}] ¡Alerta frontal! Frenando...")
                    time.sleep(0.5)

                # 4. Acumulamos el giro realizado
                total_yaw += yaw_rate * LOOP_PERIOD
                print(f"[{uri}] Giro actual: {abs(total_yaw):.1f} / 360.0 grados")

                # Si completamos los 360 grados, salimos del bucle
                if abs(total_yaw) >= 360.0:
                    print(f"[{uri}] ¡Circunferencia completa! Procediendo a aterrizar...")
                    break

                time.sleep(LOOP_PERIOD)

        except KeyboardInterrupt:
            print(f"[{uri}] Interrupción detectada. Aterrizando de forma segura...")


def main():
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache="./cache")

    print("=== INICIANDO ENJAMBRE DE ÓRBITA CIRCULAR ===")
    print("URIs configurados:")
    for uri in URI_LIST:
        print(f" - {uri}")

    with Swarm(URI_LIST, factory=factory) as swarm:
        swarm.parallel(swarm_circular_orbit)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Exiting safely...")
