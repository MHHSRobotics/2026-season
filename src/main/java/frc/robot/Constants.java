package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

import com.ctre.phoenix6.CANBus;

public final class Constants {

    // Enum for different run modes for the code
    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final Mode simMode =
            Mode.SIM; // Default simulation mode, should be SIM for regular simulation and REPLAY for replays.

    public static final Mode currentMode =
            RobotBase.isReal() ? Mode.REAL : simMode; // Current mode the robot program is in

    public static final boolean enablePhysicsSim =
            true; // Whether to enable the physics sim connector (for SIM mode only)

    public static final boolean physicsSimEnabled = currentMode == Mode.SIM
            && enablePhysicsSim; // Whether to enable the physics sim connector (for SIM mode only)

    public static final double loopTime = physicsSimEnabled ? 0.006 : 0.02; // Period of main robot loop, 20ms default

    public static final CANBus defaultBus = new CANBus("rio"); // CAN bus used for non-swerve motors

    public static final CANBus swerveBus = new CANBus("rhino"); // CAN bus used for swerve motors

    public static final double loopOverrunWarningTimeout =
            0.2; // Amount of time a robot tick can take before reporting a warning to DS

    public static final double brownoutVoltage = 6.0; // Voltage at which brownout protection occurs

    public static final double lowBatteryVoltage = 11.8; // Voltage at which low battery warning appears

    public static final double lowBatteryTime = 5; // How long to wait before reporting low battery

    public static final boolean simIsRedAlliance = false; // Whether simulated FMS is on red alliance

    public static final double simSwerveError = 0; // Simulated error in swerve odometry, set to 0 for no error

    public static final boolean ctreProLicensedWarning = true; // Whether to warn if a CTRE device isn't pro licensed

    // Teleop match time (seconds remaining) at which each SHIFT boundary occurs, per the 2026 game manual:
    // TRANSITION SHIFT 2:20-2:10, SHIFT 1 2:10-1:45, SHIFT 2 1:45-1:20, SHIFT 3 1:20-0:55, SHIFT 4 0:55-0:30,
    // END GAME 0:30-0:00
    public static final double[] shiftChangeTimes = {130, 105, 80, 55, 30};

    public static final double teleopDuration = 140; // Length of the teleop period

    public static final double[] shiftFlashWarningTimes = {10, 5
    }; // How long before a shift change to flash the warning signal

    // Color flashed for each entry in shiftFlashWarningTimes, as a parallel array
    public static final String[] shiftFlashColors = {"#FFFF00", "#FF0000"};

    public static final String shiftFlashOffColor = "#000000"; // Color shown between blinks and when no warning is
    // active

    public static final double shiftFlashDuration = 1.0; // How long each flash warning lasts

    public static final double shiftFlashBlinkRate = 5.0; // How many times per second the flash signal blinks

    // Toggles for susbsytems
    public static final boolean swerveEnabled = true;
    public static final boolean visionEnabled = true;
    public static final boolean autoAlignEnabled = false;
    public static final boolean shooterVelocityCompensationEnabled = false;
    public static final boolean shooterEnabled = false;
    public static final boolean hangEnabled = false;
    public static final boolean intakeEnabled = true;
    public static final boolean ledsEnabled = false;
}
