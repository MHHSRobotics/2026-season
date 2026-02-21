package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import frc.robot.io.MotorIO;

public class Hopper extends SubsystemBase {
    public static class Constants {
        // CAN device ID for the intake motor controller
        public static final int motorId = 18;
        // Whether to flip motor direction (true means reverse forward/backward)
        public static final boolean motorInverted = false;

        public static final double rollerSpeed = 0.5;

        public static final LoggedNetworkBoolean hopperLocked =
                new LoggedNetworkBoolean("Hopper/Locked", false); // Toggle to enable braking when stopped

        public static final LoggedNetworkBoolean hopperDisabled =
                new LoggedNetworkBoolean("Hopper/Disabled", false); // Toggle to completely disable the hopper subsystem

        public static final double rollerRatio = 1;

        // Simulation only
        public static final double rollerInertia = 0.0009; // kg m^2
    }

    private MotorIO motor;

    public Hopper(MotorIO motorIO) {
        motor = motorIO;
        motor.setInverted(Constants.motorInverted);
        motor.connectInternalSensor(Constants.rollerRatio);
    }

    public void setSpeed(double speed) {
        motor.setDutyCycle(speed);
    }

    public void forward() {
        setSpeed(Constants.rollerSpeed);
    }

    public void reverse() {
        setSpeed(-Constants.rollerSpeed);
    }

    public void stop() {
        setSpeed(0);
    }

    @Override
    public void periodic() {
        // This runs every robot loop (about 50 times per second) to update sensors and check for problems

        // Set braking based on user input
        motor.setBraking(Constants.hopperLocked.get());

        // Disable the motor based on user input
        motor.setDisabled(Constants.hopperDisabled.get());

        // Update motor inputs so the latest values are available (logging and alerts happen automatically)
        motor.update();
    }
}
