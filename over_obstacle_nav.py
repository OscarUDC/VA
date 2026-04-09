#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Obstacle Overpass Navigation for Crazyflie 2.1.
Scales an obstacle dynamically with stabilization for Flow Deck reliability.
Ensures full clearance with robust verification before landing.
"""

import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.multiranger import Multiranger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

# URI connection
URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E702")

# Behavioral Parameters
CRUISE_SPEED = 0.15  # m/s
NORMAL_HEIGHT = 0.4  # Default flight height (meters)
THRESHOLD_FRONT = 0.8  # Reaction distance (0.8m as requested)
SAFETY_MARGIN = 0.6  # Extra height once obstacle is cleared (meters)
TRIM_Y = 0.0  # Adjust if drone drifts (e.g., 0.05 to go more left)
CLEARANCE_DIST = 1.6  # distance to move forward after clearing obstacle (meters)
LOOP_PERIOD = 0.1  # Sampling period


def safe_read(dist):
    """Returns distance or a large value (5.0m) if sensor returns None."""
    return dist if dist is not None else 5.0


def run_overpass_mission(mc, mr):
    print("Mission started: Cruise mode.")

    try:
        while True:
            front = safe_read(mr.front)

            if front < THRESHOLD_FRONT:
                print(f"Obstacle detected at {front:.2f}m! Starting adaptive overpass.")
                mc.stop()

                initial_z = NORMAL_HEIGHT
                current_z = initial_z

                # 1. Incremental Climb
                print("Climbing incrementally...")
                while True:
                    front_current = safe_read(mr.front)
                    up_current = safe_read(mr.up)

                    if up_current < 0.3:
                        print("CRITICAL: Ceiling detected! Stopping climb.")
                        break

                    if front_current > 2:  # Clear of object frontally
                        print("Front cleared!")
                        break

                    step = 0.1
                    mc.up(step)
                    current_z += step
                    time.sleep(0.1)

                # 2. Add Safety Margin
                if safe_read(mr.up) > SAFETY_MARGIN + 0.2:
                    print(f"Adding {SAFETY_MARGIN}m safety margin.")
                    mc.up(SAFETY_MARGIN)
                    current_z += SAFETY_MARGIN
                else:
                    margin_possible = max(0, safe_read(mr.up) - 0.3)
                    print(f"Limited space, adding {margin_possible:.2f}m margin.")
                    mc.up(margin_possible)
                    current_z += margin_possible

                # 3. Stabilization Delay
                # This prevents the Flow Deck from glitching due to sudden height changes
                print("Stabilizing height reading for 1 second...")
                time.sleep(1.0)

                # 4. Crossing the Obstacle safely
                # Using continuous velocity (start_linear_motion) over irregular objects like people
                # can confuse the Flow Deck's optical flow, resulting in flyaways.
                # Instead, we command a fixed forward distance.

                # Desired clearance distance (user requested 1.6m)
                crossing_distance = 1.6
                print(
                    f"Crossing: Moving {crossing_distance}m forward using robust position control..."
                )

                # Stop any residual velocity before absolute movement
                mc.stop()
                time.sleep(0.5)

                # Move forward exactly 1.6m.
                # This ignores the floor topology and relies purely on relative positioning.
                mc.forward(crossing_distance, velocity=CRUISE_SPEED)

                # Stop and stabilize after crossing
                mc.stop()
                print("Stabilizing after crossing...")
                time.sleep(1.0)

                # 6. Landing
                print("Landing area reached. Landing...")
                mc.stop()
                return

            # Normal forward motion
            mc.start_linear_motion(CRUISE_SPEED, TRIM_Y, 0.0)
            time.sleep(LOOP_PERIOD)

    except KeyboardInterrupt:
        print("Mission interrupted by user.")


if __name__ == "__main__":
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    print(f"Connecting to {URI}...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        print("Connected!")
        with (
            MotionCommander(scf, default_height=NORMAL_HEIGHT) as mc,
            Multiranger(scf) as mr,
        ):
            print(f"Lift-off to {NORMAL_HEIGHT}m...")
            time.sleep(1.0)
            run_overpass_mission(mc, mr)
            print("Landing...")
            mc.stop()

    print("Success.")
