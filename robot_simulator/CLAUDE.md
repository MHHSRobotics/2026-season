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
WPILib runs at 200Hz (5ms). MuJoCo timestep is 5ms (1 physics step per control update). Stability comes from current limiting, steer frictionloss, and drive armature rather than sub-stepping. Sub-stepping to 1-2.5ms is too expensive with 408 fuel balls (~80-130Hz). Real-time pacing is enforced via wall-clock comparison.

### Motor model (`sim/simulator.py`)
Torques are computed from voltages using the DC motor equation every physics step (not just per NT update) to keep back-EMF tracking stable. Constants model a Kraken X60 FOC motor with MK4i L2 gearing (6.75:1 drive, 150/7:1 steer). Stator current is clamped to 80A (TalonFX default) to match real motor controller behavior — without this, steer torque is ~6× too high (201 Nm vs 33 Nm). MuJoCo actuators receive computed torques directly (gear=1 in XML). Robot pose and fuel poses are extracted from freejoint qpos each cycle (looked up by joint name, not hardcoded index). Fuel balls have per-step linear velocity damping (factor 0.998, linear DOFs only — angular DOFs undamped for natural rolling) applied only when on the ground (z < radius + 1cm) to simulate rolling friction without expensive condim=6 contacts.

**Implicit back-EMF damping**: The motor's back-EMF creates velocity-proportional damping (B = kt × kv × gr² / R; ~7.05 Nm·s/rad for steer, ~0.70 for drive). This damping is added to MuJoCo's `dof_damping` so it's handled by the semi-implicit integrator (stable at any timestep), then compensated in ctrl (`ctrl = motor_torque + B × ω`) to avoid double-counting. In the linear regime, physics are exact. In the current-clamped regime, MuJoCo's implicit integration prevents the discrete-time instability that otherwise causes ±33.3 Nm oscillation at 5ms timestep (the clamped braking torque would overshoot every step without this). Trade-off: slightly softened peak acceleration during current-limited transients.

### NetworkTables bridge (`sim/nt_interface.py`)
- **Reads** voltages from `AdvantageKit/Swerve/{FrontLeft,FrontRight,BackLeft,BackRight}/{Drive,Steer}/AppliedVoltage`
- **Publishes** sensor data to `MuJoCo/Swerve/{Module}/{Drive,Steer}/{Position,Velocity,Angle}` and `MuJoCo/Gyro/{Yaw,Pitch,Roll}`
- **Publishes** robot pose as struct-encoded `Pose3d` to `MuJoCo/Swerve/Pose`
- **Publishes** fuel ball poses as struct array of `Pose3d` to `MuJoCo/Fuel` (48 balls, 50ms periodic)
- All publishers/subscribers use `periodic=0.02` (20ms) and `flush()` after each update to avoid NT4's default 100ms batching

### MuJoCo models
- `models/robot.xml` - 27.5" square frame with bumpers, ~63.5kg (140 lbs) total. Chassis is thin (top 0.097m, bottom 0.043m above floor, center at z=0.07). Four swerve modules at corners (0.28m offset). Wheels are 0.05m radius cylinders. Chassis has a freejoint for 6DOF movement. Steer joints have damping=1.0, armature=0.011 (reflected rotor inertia), frictionloss=2.0 (MK4i bevel gear friction). Drive joints have armature=0.003 (reflected rotor inertia through 6.75:1). Includes field via `<include>`. Front-mounted intake assembly pivots on a hinge (0.051m behind frame edge, 0.197m above floor, range -90° to 0°, starts stowed at -90°) with a spinning roller (0.1m dia, 0.455m wide, 0.228m in front of frame edge), backing bar, side bars, and polycarbonate shields (0.312m tall). Hopper with 4 coupled rollers (0.051m dia, along Y from -0.232 to 0.175, dark green) at ascending heights (0.126–0.147m above floor) driven by a single motor via equality constraints, flanked by two 0.035m-thick gold walls (0.097–0.488m above floor). Double shooter at back of frame: U-shaped aluminum base (2" high, 1" wide blocks), 3 polycarb walls (left/right/center, 0.33m tall, 0.006m thick), 4 coupled dark blue feed wheels (0.0566m dia, 0.025m thick), 2 independent white flywheels (0.1m dia, 0.052m thick), and 11 churros forming a curved ball track (curves backward then forward at the top to direct balls forward). Self-collision between all robot parts is prevented via contype=2/conaffinity=1 (field/fuel use contype=1/conaffinity=1).
- `models/robot.xml` includes `<visual><headlight ambient="0.5 0.5 0.5"/></visual>` for better lighting.
- `models/field.xml` - 2026 REBUILT field using colored collision primitives (no mesh overlay, no STLs). WPILib coordinate convention (origin at blue alliance wall corner). Blue/red halves are exact 180° rotations about field center (8.27, 4.035). Alliance-colored elements: walls (`blue_wall_mat`/`red_wall_mat`), tower posts/supports/bumps/trench posts/trench walls/corral walls (`blue_accent_mat`/`red_accent_mat`), hub funnel/gap panels/ramp (`blue_hub_mat`/`red_hub_mat`). Neutral elements: hub walls (`hub_mat`), guardrails/corral ramps (`wall_mat`), carpet (`carpet_mat`), depots (`depot_mat`), rungs (`rung_mat` orange), nets (`net_mat` semi-transparent). Floor has explicit friction=1.0 for wheel grip. Fuel: 408 free-body yellow spheres (5.91" dia, 0.14kg) with `class="fuel"` default (friction 0.8, condim=3) — 24 per depot (6x4 grids) + 360 in neutral zone (15x24 grid centered at field center, 15 along X, 24 along Y).

## Key conventions

- Module keys: `fl`, `fr`, `bl`, `br` internally; `FrontLeft`, `FrontRight`, `BackLeft`, `BackRight` in NT paths
- All angles in radians, all voltages clamped to [-12, 12]V
- Sensor data uses MuJoCo sensor API (indexed by `sensor_adr`), not direct joint reads
- NT topic names should NOT have a leading `/` (NT4 convention). Some NT clients normalize this inconsistently.
- Steer joints use damping=1.0 (XML) + frictionloss=2.0 to model MK4i bevel gear friction. Back-EMF damping (~7.05 steer, ~0.70 drive) is added at runtime to `dof_damping` for implicit integration stability.
- Motor current is clamped to ±80A (TalonFX stator current limit). Ctrl signal adds `B × ω` to compensate for implicit back-EMF damping.
- MuJoCo combines contact friction as element-wise **max** of both geoms. Default geom friction is 0.3 (low, for walls/structural elements). Floor has explicit friction=1.0, wheels 1.5, rollers/flywheels 2.0, bumpers 0.4, fuel 0.8. This keeps wall-bumper friction realistic (~0.4) while preserving wheel-carpet grip.

## Environment

- Project lives on Windows, accessed from WSL via `/mnt/c/...`
- Python/conda environment runs on Windows side; use `/mnt/c/Windows/System32/cmd.exe /c "conda activate frc-mujoco-sim && ..."` to run Python
- For multi-line Python, write a temp script and run it rather than fighting cmd.exe quoting
