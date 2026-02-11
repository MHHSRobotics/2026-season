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
    # WPILib runs at 50Hz (20ms), MuJoCo at 1000Hz (1ms)
    # So we run 20 physics steps per control update
    wpilib_period = 0.005  # 20ms
    physics_steps_per_update = int(wpilib_period / sim.get_timestep())

    print(f"Running {physics_steps_per_update} physics steps per control update")
    print("Simulation running. Close viewer window to exit.")
    if args.standalone:
        print("(Standalone mode - no NetworkTables)")

    try:
        last_update_time = time.time()
        last_print_time = 0.0

        while True:
            # Check if viewer was closed
            if not args.no_viewer and not sim.viewer_is_running():
                print("Viewer closed, exiting...")
                break

            current_time = time.time()
            dt = current_time - last_update_time

            # Run at approximately real-time
            if dt >= wpilib_period:
                last_update_time = current_time

                # Get inputs from NetworkTables
                if nt is not None:
                    inputs = nt.get_inputs()
                    sim.set_voltages(inputs)

                # Step physics
                sim.step(physics_steps_per_update)

                # Send outputs to NetworkTables
                if nt is not None:
                    outputs = sim.get_outputs()
                    nt.set_outputs(outputs)

                # Print status periodically
                if current_time - last_print_time >= 0.5:
                    last_print_time = current_time
                    connected = "connected" if nt is not None and nt.is_connected() else "disconnected"
                    if nt is not None:
                        print(
                            f"[{connected}] "
                            f"FL({inputs.fl.drive_voltage:+5.1f}V/{inputs.fl.steer_voltage:+5.1f}V) "
                            f"FR({inputs.fr.drive_voltage:+5.1f}V/{inputs.fr.steer_voltage:+5.1f}V) "
                            f"BL({inputs.bl.drive_voltage:+5.1f}V/{inputs.bl.steer_voltage:+5.1f}V) "
                            f"BR({inputs.br.drive_voltage:+5.1f}V/{inputs.br.steer_voltage:+5.1f}V)",
                            end="\r",
                        )

                # Update viewer
                if not args.no_viewer:
                    sim.sync_viewer()
            else:
                # Sleep to avoid busy-waiting
                time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nInterrupted, exiting...")
    finally:
        if nt is not None:
            nt.close()


if __name__ == "__main__":
    main()
