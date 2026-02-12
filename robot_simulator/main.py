#!/usr/bin/env python3
"""Main entry point for the FRC swerve robot simulator."""

import argparse
import math
import time
from pathlib import Path

from sim import SwerveSimulator, NetworkTablesInterface


def main():
    parser = argparse.ArgumentParser(description="FRC Swerve Robot MuJoCo Simulator")
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(__file__).parent / "models" / "robot.xml",
        help="Path to MuJoCo model XML file",
    )
    parser.add_argument(
        "--nt-server",
        type=str,
        default="localhost",
        help="NetworkTables server address",
    )
    parser.add_argument(
        "--nt-port",
        type=int,
        default=5810,
        help="NetworkTables port (default: 5810 for WPILib sim)",
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="Run without visualization (headless mode)",
    )
    parser.add_argument(
        "--standalone",
        action="store_true",
        help="Run without NetworkTables (for testing)",
    )
    args = parser.parse_args()

    # Initialize simulator
    print(f"Loading model from: {args.model}")
    sim = SwerveSimulator(args.model)
    print(f"Simulation timestep: {sim.get_timestep() * 1000:.1f} ms")

    # Initialize NetworkTables (unless standalone mode)
    nt = None
    if not args.standalone:
        print(f"Connecting to NetworkTables at {args.nt_server}:{args.nt_port}")
        nt = NetworkTablesInterface(args.nt_server, args.nt_port)

    # Launch viewer
    if not args.no_viewer:
        print("Launching viewer...")
        sim.launch_viewer()

    # Simulation parameters
    # WPILib runs at 200Hz (5ms), MuJoCo at 200Hz (5ms)
    # 1 physics step per control update; stability comes from current limiting,
    # steer frictionloss, and drive armature rather than sub-stepping
    wpilib_period = 0.005  # 5ms
    physics_steps_per_update = int(wpilib_period / sim.get_timestep())

    print(f"Running {physics_steps_per_update} physics steps per control update")
    print("Simulation running. Close viewer window to exit.")
    if args.standalone:
        print("(Standalone mode - no NetworkTables)")

    try:
        next_update_time = time.time()
        last_print_time = next_update_time
        frame_count = 0

        while True:
            # Check if viewer was closed
            if not args.no_viewer and not sim.viewer_is_running():
                print("Viewer closed, exiting...")
                break

            now = time.time()

            # Sleep until next update is due
            sleep_time = next_update_time - now
            if sleep_time > 0.001:
                time.sleep(sleep_time - 0.0005)  # wake slightly early to avoid oversleep
                continue
            elif sleep_time > 0:
                continue  # busy-wait the last bit

            # Schedule next update (advance by period, don't accumulate drift)
            # If we're behind, skip ahead to avoid spiral of death
            if now - next_update_time > wpilib_period * 10:
                next_update_time = now
            next_update_time += wpilib_period

            # Get inputs from NetworkTables
            if nt is not None:
                inputs = nt.get_inputs()
                sim.set_voltages(inputs)

            # Step physics
            sim.step(physics_steps_per_update)
            frame_count += 1

            # Send outputs to NetworkTables
            if nt is not None:
                outputs = sim.get_outputs()
                nt.set_outputs(outputs)

            # Print status periodically
            if now - last_print_time >= 0.5:
                fps = frame_count / (now - last_print_time)
                frame_count = 0
                last_print_time = now
                connected = "connected" if nt is not None and nt.is_connected() else "disconnected"
                if nt is not None:
                    print(
                        f"[{connected}] {fps:5.1f} Hz  "
                        f"FL({inputs.fl.drive_voltage:+5.1f}V/{inputs.fl.steer_voltage:+5.1f}V) "
                        f"FR({inputs.fr.drive_voltage:+5.1f}V/{inputs.fr.steer_voltage:+5.1f}V) "
                        f"BL({inputs.bl.drive_voltage:+5.1f}V/{inputs.bl.steer_voltage:+5.1f}V) "
                        f"BR({inputs.br.drive_voltage:+5.1f}V/{inputs.br.steer_voltage:+5.1f}V)",
                        end="\r",
                    )
                else:
                    print(f"{fps:5.1f} Hz", end="\r")

            # Update viewer
            if not args.no_viewer:
                sim.sync_viewer()

    except KeyboardInterrupt:
        print("\nInterrupted, exiting...")
    finally:
        if nt is not None:
            nt.close()


if __name__ == "__main__":
    main()
