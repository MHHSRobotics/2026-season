# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

MuJoCo-based physics simulator for an FRC swerve drive robot. The simulator runs a MuJoCo physics engine and communicates with a WPILib robot program via NetworkTables, acting as a hardware-in-the-loop substitute for real robot hardware.

## Running

```bash
# With NetworkTables (connects to WPILib sim on localhost:5810)
python main.py

# Standalone test (no NetworkTables, full-throttle drive with PD steering)
python test_drive.py

# Headless / custom options
python main.py --standalone --no-viewer --nt-server <ip> --nt-port <port>
```

## Dependencies

Python 3.11, managed via conda (`environment.yml`) or pip (`requirements.txt`):
- `mujoco>=3.0.0` - physics engine
- `pyntcore>=2024.0.0` - NetworkTables client
- `robotpy-wpimath>=2024.0.0` - WPILib math types (Pose3d, etc.) for struct-encoded NT publishing
- `numpy>=1.24`

## Architecture

### Simulation loop (`main.py`)
WPILib runs at 200Hz (5ms). MuJoCo timestep is 2ms. Each control update runs physics steps to match. Real-time pacing is enforced via wall-clock comparison.

### Motor model (`sim/simulator.py`)
Torques are computed from voltages using the DC motor equation every physics step (not just per NT update) to keep back-EMF tracking stable. Constants model a Kraken X60 FOC motor with MK4i L2 gearing (6.75:1 drive, 150/7:1 steer). MuJoCo actuators receive computed torques directly (gear=1 in XML). Robot pose and fuel poses are extracted from freejoint qpos each cycle (looked up by joint name, not hardcoded index). Fuel balls have per-step velocity damping (factor 0.998) applied only when on the ground (z < radius + 1cm) to simulate rolling friction without expensive condim=6 contacts.

### NetworkTables bridge (`sim/nt_interface.py`)
- **Reads** voltages from `AdvantageKit/Swerve/{FrontLeft,FrontRight,BackLeft,BackRight}/{Drive,Steer}/AppliedVoltage`
- **Publishes** sensor data to `MuJoCo/Swerve/{Module}/{Drive,Steer}/{Position,Velocity,Angle}` and `MuJoCo/Gyro/{Yaw,Pitch,Roll}`
- **Publishes** robot pose as struct-encoded `Pose3d` to `MuJoCo/Swerve/Pose`
- **Publishes** fuel ball poses as struct array of `Pose3d` to `MuJoCo/Fuel` (48 balls, 50ms periodic)
- All publishers/subscribers use `periodic=0.02` (20ms) and `flush()` after each update to avoid NT4's default 100ms batching

### MuJoCo models
- `models/robot.xml` - 27.5" square frame with bumpers, ~48kg total. Four swerve modules at corners (0.28m offset). Wheels are 0.05m radius cylinders. Chassis has a freejoint for 6DOF movement. Includes field via `<include>`.
- `models/robot.xml` includes `<visual><headlight ambient="0.5 0.5 0.5"/></visual>` for better lighting.
- `models/field.xml` - 2026 REBUILT field using colored collision primitives (no mesh overlay, no STLs). WPILib coordinate convention (origin at blue alliance wall corner). Blue/red halves are exact 180° rotations about field center (8.27, 4.035). Alliance-colored elements: walls (`blue_wall_mat`/`red_wall_mat`), tower posts/supports/bumps/trench posts/trench walls/corral walls (`blue_accent_mat`/`red_accent_mat`), hub funnel/gap panels/ramp (`blue_hub_mat`/`red_hub_mat`). Neutral elements: hub walls (`hub_mat`), guardrails/corral ramps (`wall_mat`), carpet (`carpet_mat`), depots (`depot_mat`), rungs (`rung_mat` orange), nets (`net_mat` semi-transparent). Fuel: 408 free-body yellow spheres (5.91" dia, 0.14kg) with `class="fuel"` default (friction 0.8, condim=3) — 24 per depot (6x4 grids) + 360 in neutral zone (15x24 grid centered at field center, 15 along X, 24 along Y).

## Key conventions

- Module keys: `fl`, `fr`, `bl`, `br` internally; `FrontLeft`, `FrontRight`, `BackLeft`, `BackRight` in NT paths
- All angles in radians, all voltages clamped to [-12, 12]V
- Sensor data uses MuJoCo sensor API (indexed by `sensor_adr`), not direct joint reads
- NT topic names should NOT have a leading `/` (NT4 convention). Some NT clients normalize this inconsistently.
- Steer joints use elevated damping (5.0) to compensate for NT transport delay destabilizing high-bandwidth PD controllers

## Environment

- Project lives on Windows, accessed from WSL via `/mnt/c/...`
- Python/conda environment runs on Windows side; use `/mnt/c/Windows/System32/cmd.exe /c "conda activate frc-mujoco-sim && ..."` to run Python
- For multi-line Python, write a temp script and run it rather than fighting cmd.exe quoting
