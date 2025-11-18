package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import frc.robot.io.MotorIO;

// Make the intake subsystem control a simple motor for picking up and ejecting game pieces.
// This is just basic speed control - no PID, no simulation, just intake/outtake movement.
public class Intake extends SubsystemBase {

    public static class Constants {
        // CAN device ID for the intake motor controller
        public static final int motorId = 24;
        // Whether to flip motor direction (true means reverse forward/backward)
        public static final boolean motorInverted = true;

        public static final double statorCurrentLimit = 60; // (amps) limit on motor torque output for intake loads
        public static final double supplyCurrentLimit = 50; // (amps) normal current limit pulled from battery
        public static final double supplyCurrentLowerLimit = 35; // (amps) reduce to this if over limit for some time
        public static final double supplyCurrentLowerTime = 0.3; // (seconds) time before lowering current limit

        public static final LoggedNetworkBoolean intakeLocked = new LoggedNetworkBoolean(
                "Intake/Locked", true); // Toggle to enable braking when stopped (usually false for intake)

        public static final LoggedNetworkBoolean intakeDisabled =
                new LoggedNetworkBoolean("Intake/Disabled", false); // Toggle to completely disable the intake subsystem
    }

    // Intake motor interface; handles real robot only (no simulation)
    private MotorIO motor;

    public Intake(MotorIO motorIO) {
        motor = motorIO;

        // Tell the motor which direction is forward (true = invert)
        motor.setInverted(Constants.motorInverted);

        // Set up current limits for intake loads
        motor.setStatorCurrentLimit(Constants.statorCurrentLimit);
        motor.setSupplyCurrentLimit(Constants.supplyCurrentLimit);
        motor.setSupplyCurrentLowerLimit(Constants.supplyCurrentLowerLimit);
        motor.setSupplyCurrentLowerTime(Constants.supplyCurrentLowerTime);
    }

    // Tell the intake motor how fast to spin (percent [-1 to 1], -1 = full outtake, 1 = full intake)
    public void setSpeed(double speed) {
        motor.setDutyCycle(speed);
    }

    @Override
    public void periodic() {
        // This runs every robot loop (about 50 times per second) to update sensors and check for problems

        // Set braking based on user input (usually false for intake)
        motor.setBraking(Constants.intakeLocked.get());

        // Disable the motor based on user input
        motor.setDisabled(Constants.intakeDisabled.get());

        // Update motor inputs so the latest values are available (logging and alerts happen automatically)
        motor.update();
    }
}
