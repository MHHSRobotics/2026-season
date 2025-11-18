package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import frc.robot.io.MotorIO;

// Make the hang subsystem control a simple motor for climbing at the end of the match.
// This is just basic speed control - no PID, no simulation, just up/down movement.
public class Hang extends SubsystemBase {

    public static class Constants {
        // CAN device ID for the hang motor controller
        public static final int motorId = 25;
        // Whether to flip motor direction (true means reverse forward/backward)
        public static final boolean motorInverted = false;

        public static final double statorCurrentLimit = 80; // (amps) limit on motor torque output for climbing
        public static final double supplyCurrentLimit = 70; // (amps) normal current limit pulled from battery
        public static final double supplyCurrentLowerLimit = 50; // (amps) reduce to this if over limit for some time
        public static final double supplyCurrentLowerTime = 0.5; // (seconds) time before lowering current limit

        public static final LoggedNetworkBoolean hangLocked =
                new LoggedNetworkBoolean("Hang/Locked", true); // Toggle to enable braking when stopped

        public static final LoggedNetworkBoolean hangDisabled =
                new LoggedNetworkBoolean("Hang/Disabled", false); // Toggle to completely disable the hang subsystem
    }

    // Hang motor interface; handles real robot only (no simulation)
    private MotorIO motor;

    public Hang(MotorIO motorIO) {
        motor = motorIO;

        // Tell the motor which direction is forward (true = invert)
        motor.setInverted(Constants.motorInverted);

        // Set up current limits for climbing loads
        motor.setStatorCurrentLimit(Constants.statorCurrentLimit);
        motor.setSupplyCurrentLimit(Constants.supplyCurrentLimit);
        motor.setSupplyCurrentLowerLimit(Constants.supplyCurrentLowerLimit);
        motor.setSupplyCurrentLowerTime(Constants.supplyCurrentLowerTime);
    }

    // Tell the hang motor how fast to spin (percent [-1 to 1], -1 = full down, 1 = full up)
    public void setSpeed(double speed) {
        motor.setDutyCycle(speed);
    }

    @Override
    public void periodic() {
        // This runs every robot loop (about 50 times per second) to update sensors and check for problems

        // Set braking based on user input
        motor.setBraking(Constants.hangLocked.get());

        // Disable the motor based on user input
        motor.setDisabled(Constants.hangDisabled.get());

        // Update motor inputs so the latest values are available (logging and alerts happen automatically)
        motor.update();
    }
}
