#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Complex Route for Crazyflie 2.1 + Flow Deck + Multi-Ranger Deck.
This script implements a state-machine based navigation for autonomous flight.
It can search for walls, follow a single wall, or stay centered in a corridor.
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

# URI connection
URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E702")

# Behavioral Parameters
SEARCH_SPEED = 0.15  # m/s during wall searching
FOLLOW_SPEED = 0.10  # m/s during wall/corridor following
BACK_SPEED = 0.15  # m/s during evasion
TURN_ANGLE = 45  # degrees to turn during evasion
LOOP_PERIOD = 0.1  # s between iterations (10 Hz)
THRESHOLD_FRONT = 0.4  # m, front collision threshold
THRESHOLD_SIDE = 0.3  # m, target distance to side wall
KP_SIDE = 0.6  # Proportional gain for lateral correction
MAX_VY = 0.2  # Max lateral speed


def safe_read(dist):
    """Returns distance or a large value (5m) if sensor returns None."""
    return dist if dist is not None else 5.0


class State:
    SEARCHING = "SEARCHING"
    FOLLOW_RIGHT = "FOLLOW_RIGHT"
    FOLLOW_LEFT = "FOLLOW_LEFT"
    CORRIDOR = "CORRIDOR"
    EVASION = "EVASION"


def run_mission(mc, mr):
    state = State.SEARCHING
    print(f"Starting mission in state: {state}")

    try:
        while True:
            # Read sensors
            front = safe_read(mr.front)
            left = safe_read(mr.left)
            right = safe_read(mr.right)

            # --- Global Evasion Logic ---
            if front < THRESHOLD_FRONT:
                print(f"CRITICAL: Front obstacle at {front:.2f}m. Evading!")
                mc.stop()
                mc.back(0.15, velocity=BACK_SPEED)
                # Determine turn direction based on side sensors
                if left > right:
                    mc.turn_left(TURN_ANGLE)
                else:
                    mc.turn_right(TURN_ANGLE)
                time.sleep(0.5)
                state = State.SEARCHING
                continue

            # --- State Machine ---
            if state == State.SEARCHING:
                # If we detect walls on both sides, enter corridor mode
                if left < 1.0 and right < 1.0:
                    state = State.CORRIDOR
                    print("Transition: SEARCHING -> CORRIDOR")
                # If only right wall detected
                elif right < 0.6:
                    state = State.FOLLOW_RIGHT
                    print("Transition: SEARCHING -> FOLLOW_RIGHT")
                # If only left wall detected
                elif left < 0.6:
                    state = State.FOLLOW_LEFT
                    print("Transition: SEARCHING -> FOLLOW_LEFT")
                else:
                    # Keep moving forward to search
                    mc.start_linear_motion(SEARCH_SPEED, 0.0, 0.0)

            elif state == State.FOLLOW_RIGHT:
                if left < 0.8:  # Detected other wall
                    state = State.CORRIDOR
                    continue
                if right > 1.2:  # Lost right wall
                    print("Lost right wall, searching...")
                    state = State.SEARCHING
                    continue

                error = THRESHOLD_SIDE - right
                vy = np.clip(error * KP_SIDE, -MAX_VY, MAX_VY)
                mc.start_linear_motion(FOLLOW_SPEED, vy, 0.0)

            elif state == State.FOLLOW_LEFT:
                if right < 0.8:  # Detected other wall
                    state = State.CORRIDOR
                    continue
                if left > 1.2:  # Lost left wall
                    print("Lost left wall, searching...")
                    state = State.SEARCHING
                    continue

                # For left wall, positive error means too close, need to move right (positive vy)
                error = left - THRESHOLD_SIDE
                vy = np.clip(error * KP_SIDE, -MAX_VY, MAX_VY)
                mc.start_linear_motion(FOLLOW_SPEED, vy, 0.0)

            elif state == State.CORRIDOR:
                if left > 1.0 and right > 1.0:
                    state = State.SEARCHING
                    print("Corridor ended, searching...")
                    continue

                # Centering logic: target (left-right) to be 0
                error = (left - right) / 2.0
                vy = np.clip(error * KP_SIDE, -MAX_VY, MAX_VY)
                mc.start_linear_motion(FOLLOW_SPEED, vy, 0.0)

            time.sleep(LOOP_PERIOD)

    except KeyboardInterrupt:
        print("Mission interrupted by user.")


if __name__ == "__main__":
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    print(f"Connecting to {URI}...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        with MotionCommander(scf, default_height=0.4) as mc, Multiranger(scf) as mr:
            print("Lift-off! Waiting for stability...")
            time.sleep(1.0)
            run_mission(mc, mr)
            print("Landing...")
            mc.stop()

    print("Mission completed.")
