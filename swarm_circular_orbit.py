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

# Shared behavior parameters (same as single-drone script)
DEFAULT_HEIGHT = 0.4
FORWARD_SPEED = 0.15
THRESHOLD_FRONT = 0.3
THRESHOLD_SIDE = 0.3
KP_YAW = 25.0
LOOP_PERIOD = 0.1

SIDE_DEADBAND = 0.05
MAX_YAW_RATE = 55.0

SIDE_LOST_DISTANCE = 1.2
LOST_CYCLES_TO_RECOVER = 3
RECOVERY_YAW_RATE = 24.0
RECOVERY_REORIENT_YAW_RATE = 32.0
RECOVERY_FORWARD_SPEED = 0.04
FRONT_REORIENT_THRESHOLD = 0.45
FRONT_CONFIRM_CYCLES = 2
REVERSE_REORIENT_CYCLES = 6
REACQUIRE_BAND = 0.12
REACQUIRE_STABLE_CYCLES = 2
FRONT_HARD_STOP = 0.05


def safe_read(dist):
    return dist if dist is not None else 4.0


def swarm_circular_orbit(scf):
    uri = scf.cf.link_uri
    print(f"[{uri}] Starting circular orbit behavior")

    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc, Multiranger(scf) as mr:
        time.sleep(1.0)

        # Phase 1: approach front object
        print(f"[{uri}] Searching object at front...")
        while safe_read(mr.front) > THRESHOLD_FRONT:
            mc.start_linear_motion(0.1, 0.0, 0.0)
            time.sleep(LOOP_PERIOD)

        mc.stop()
        print(f"[{uri}] Object found. Moving object to right side...")

        # Phase 2: initial orientation for right-side orbit
        mc.turn_left(90)
        time.sleep(1.0)

        # Phase 3: orbit loop
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

                    error_side = THRESHOLD_SIDE - dist_right
                    if abs(error_side) < SIDE_DEADBAND:
                        error_side = 0.0

                    yaw_rate = error_side * KP_YAW
                    yaw_rate = float(np.clip(yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE))

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
                        print(f"[{uri}] Side reference lost -> RECOVERY")

            else:  # RECOVERY
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

                if side_visible and abs(dist_right - THRESHOLD_SIDE) <= REACQUIRE_BAND:
                    reacquire_counter += 1
                else:
                    reacquire_counter = 0

                if reacquire_counter >= REACQUIRE_STABLE_CYCLES:
                    mode = "ORBIT"
                    lost_counter = 0
                    print(f"[{uri}] Side reference recovered -> ORBIT")

            if dist_front < FRONT_HARD_STOP:
                mc.stop()
                print(f"[{uri}] Front hard stop triggered")
                time.sleep(0.3)

            time.sleep(LOOP_PERIOD)


def main():
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache="./cache")

    print("=== Starting SWARM circular orbit ===")
    print("Configured URIs:")
    for uri in URI_LIST:
        print(f" - {uri}")

    with Swarm(URI_LIST, factory=factory) as swarm:
        swarm.parallel(swarm_circular_orbit)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Exiting safely...")
