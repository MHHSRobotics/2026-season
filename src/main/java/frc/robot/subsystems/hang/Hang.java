package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import frc.robot.io.MotorIO;


public class Hang extends SubsystemBase{
    public static class Constants {
        // CAN device ID for the hang motor controller, and Digital input sensors initialized.
        private DigitalInput irSensor1;
        private DigitalInput irSensor2;
        private DigitalInput irSensor3;
        public static final int motorId = 0;

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

    private MotorIO motorR;
    private MotorIO motorL;

    public boolean invert = false;

    public Hang(MotorIO motorIO) {
        motorR.setInverted(Constants.motorInverted);
        motorL.setInverted(Constants.motorInverted);

        motorR.setStatorCurrentLimit(Constants.statorCurrentLimit);
        motorR.setSupplyCurrentLimit(Constants.supplyCurrentLimit);
        motorR.setSupplyCurrentLowerLimit(Constants.supplyCurrentLowerLimit);
        motorR.setSupplyCurrentLowerTime(Constants.supplyCurrentLowerTime);

        motorL.setStatorCurrentLimit(Constants.statorCurrentLimit);
        motorL.setSupplyCurrentLimit(Constants.supplyCurrentLimit);
        motorL.setSupplyCurrentLowerLimit(Constants.supplyCurrentLowerLimit);
        motorL.setSupplyCurrentLowerTime(Constants.supplyCurrentLowerTime);
    }

    public void setSpeedLeft(double speed) {
        motorL.setDutyCycle(speed);
    }

    public void setSpeedRight(double speed) {
        motorR.setDutyCycle(speed);
    }

    public void setSpeed(double speed) {
        motorR.setDutyCycle(speed);
        motorL.setDutyCycle(speed);
    }

    @Override
    public void periodic() {
        // This runs every robot loop (about 50 times per second) to update sensors and check for problems

        // Set braking based on user input
        motorL.setBraking(Constants.hangLocked.get());

        // Disable the motor based on user input
        motorL.setDisabled(Constants.hangDisabled.get());

        // Update motor inputs so the latest values are available (logging and alerts happen automatically)
        motorL.update();
    }
}
