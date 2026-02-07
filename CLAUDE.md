# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
./gradlew build              # Build the project
./gradlew deploy             # Deploy to robot (requires robot connection)
./gradlew simulateJava       # Run simulation with GUI
./gradlew spotlessApply      # Format code (Palantir Java Format)
./gradlew spotlessCheck      # Check formatting without fixing
```

## Architecture Overview

This is Team 5137's FRC 2026 robot code using WPILib command-based framework with AdvantageKit logging.

### IO Abstraction Pattern

Hardware is abstracted through IO interfaces in `src/main/java/frc/robot/io/`:

- **MotorIO** - Base motor interface with implementations for TalonFX, TalonFXS, SparkMax
- **EncoderIO** - Absolute encoder interface (CANcoder implementation)
- **GyroIO** - IMU interface (Pigeon 2 implementation)
- **CameraIO** - Vision camera interface (PhotonVision implementation)

Each IO class has an `Inputs` inner class with `@AutoLog` annotation for automatic AdvantageKit logging. Subsystems accept IO interfaces, enabling:
- Real hardware in REAL/SIM modes
- Empty IO objects in REPLAY mode for log playback
- Easy hardware swapping without subsystem changes

### Mode System

`Constants.currentMode` controls initialization behavior:
- **REAL** - Running on robot with real hardware
- **SIM** - Simulation with physics models
- **REPLAY** - Log playback through simulation

### Subsystem Structure

**Swerve** (`subsystems/swerve/`):
- 4 modules (FL, FR, BL, BR) each with drive motor, steer motor, CANcoder
- `TunerConstants.java` - Auto-generated from Phoenix Tuner X, contains all swerve gains
- `SwerveSim.java` - Physics simulation with optional error injection
- Uses "rhino" CAN bus (CANivore) for lower latency

**Shooter** (`subsystems/shooter/`):
- Feed motor, flywheel motor, pitch motor with encoder
- Currently has placeholder constants (needs tuning)

### Commands

Commands are organized in static factory classes (`SwerveCommands`, `ShooterCommands`) that take subsystem references and return `Command` objects.

### Dynamic Tuning

PID gains use `LoggedNetworkNumber` for live dashboard adjustment:
```java
LoggedNetworkNumber driveKP = new LoggedNetworkNumber("Swerve/DriveKP", 0.1);
```
Values persist in logs and can be modified via NetworkTables without redeploying.

### CAN Bus Configuration

- **"rio"** - Default RoboRIO CAN bus
- **"rhino"** - CANivore bus for swerve (all drive/steer motors, CANcoders, Pigeon)

### Key Constants

- Loop time: 20ms (50Hz)
- Swerve max speed: 5.04 m/s
- Brownout voltage: 6.0V
- Low battery warning: 11.8V

### Feature Flags

In `Constants.java`:
- `swerveEnabled` - Enable/disable swerve subsystem
- `visionEnabled` - Enable/disable vision pose estimation (currently false)
