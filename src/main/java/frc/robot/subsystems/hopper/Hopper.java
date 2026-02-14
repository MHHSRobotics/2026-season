package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.io.MotorIO;

public class Hopper extends SubsystemBase {
    public static class Constants {
        // CAN device ID for the intake motor controller
        public static final int motorId = 24;
        // Whether to flip motor direction (true means reverse forward/backward)
        public static final boolean motorInverted = false;

        public static final double rollerSpeed = 0.5;
    }

    private MotorIO motor;

    public Hopper(MotorIO motorIO) {
        motor = motorIO;
        motor.setInverted(Constants.motorInverted);
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
}
