package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import frc.robot.io.MotorIO;


public class Hang extends SubsystemBase{
    public static class Constants {
        // CAN device ID for the hang motor controller, and Digital input sensors initialized.
        public static final int motorId = 25;

        // Whether to flip motor direction (true means reverse forward/backward)
        public static final boolean motorInverted = false;

        public static final LoggedNetworkBoolean hangLocked =
                new LoggedNetworkBoolean("Hang/Locked", true); // Toggle to enable braking when stopped

        public static final LoggedNetworkBoolean hangDisabled =
                new LoggedNetworkBoolean("Hang/Disabled", false); // Toggle to completely disable the hang subsystem
    }

    private MotorIO hangMotor;

    public Hang(MotorIO motorIO) {
        hangMotor.setInverted(Constants.motorInverted);
    }

    public void setSpeed(double speed) {
        hangMotor.setDutyCycle(speed);
    }

    @Override
    public void periodic() {
        // This runs every robot loop (about 50 times per second) to update sensors and check for problems

        // Set braking based on user input
        hangMotor.setBraking(Constants.hangLocked.get());

        // Disable the motor based on user input
        hangMotor.setDisabled(Constants.hangDisabled.get());

        // Update motor inputs so the latest values are available (logging and alerts happen automatically)
        hangMotor.update();
    }
}
