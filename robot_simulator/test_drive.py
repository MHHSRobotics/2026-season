"""Test script: drives the robot forward at full throttle to verify physics.

Uses a simple PID loop on steering to hold wheels at 0 degrees,
simulating what WPILib would do in normal operation.
"""

import mujoco.viewer
import numpy as np
import time
from sim import SwerveSimulator, SwerveInputs
from sim.nt_interface import SwerveModuleState

# Simple steering PID gains
STEER_KP = 12.0  # V/rad
STEER_KD = 1.0   # V/(rad/s)


def steer_pid(angle: float, target: float = 0.0, velocity: float = 0.0) -> float:
    """Simple PD controller returning a voltage to hold steering at target angle."""
    error = target - angle
    return np.clip(STEER_KP * error - STEER_KD * velocity, -12, 12)


def main():
    sim = SwerveSimulator("models/robot.xml")

    print("Full throttle drive test with PID steering hold (close viewer to exit)")
    print(f"{'Time':>5s}  {'Speed':>7s}  {'X pos':>7s}  {'FL steer':>8s}")

    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        wall_start = time.time()
        last_print = 0

        while viewer.is_running():
            out = sim.get_outputs()

            # PID steering hold + full throttle drive
            inputs = SwerveInputs(
                fl=SwerveModuleState(
                    steer_voltage=steer_pid(out.fl_steer_angle),
                    drive_voltage=12,
                ),
                fr=SwerveModuleState(
                    steer_voltage=steer_pid(out.fr_steer_angle),
                    drive_voltage=12,
                ),
                bl=SwerveModuleState(
                    steer_voltage=steer_pid(out.bl_steer_angle),
                    drive_voltage=12,
                ),
                br=SwerveModuleState(
                    steer_voltage=steer_pid(out.br_steer_angle),
                    drive_voltage=12,
                ),
            )

            sim.set_voltages(inputs)
            sim.step(1)
            viewer.sync()

            if sim.data.time - last_print >= 0.5:
                speed = np.sqrt(sim.data.qvel[0] ** 2 + sim.data.qvel[1] ** 2)
                x_pos = sim.data.qpos[0]
                fl_steer = np.degrees(out.fl_steer_angle)
                print(
                    f"{sim.data.time:5.1f}s  {speed:5.2f}m/s  {x_pos:+6.2f}m  {fl_steer:+7.1f}deg"
                )
                last_print = sim.data.time

            # Real-time pacing
            elapsed = time.time() - wall_start
            if sim.data.time > elapsed:
                time.sleep(sim.data.time - elapsed)

    print("Done")


if __name__ == "__main__":
    main()
