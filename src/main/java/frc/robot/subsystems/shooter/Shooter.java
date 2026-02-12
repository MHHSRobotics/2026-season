package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import frc.robot.io.MotorIO;

public class Shooter extends SubsystemBase {
    public static class Constants {
        // Subsystem Constants
        public static final LoggedNetworkBoolean shooterLocked = new LoggedNetworkBoolean("Shooter/Locked", false);
        public static final LoggedNetworkBoolean shooterDisabled = new LoggedNetworkBoolean("Shooter/Disabled", false);

        public static final double feedSpeed = 0.5;

        // flyConstants
        public static final double flyGearRatio = 20;

        public static final double flySpeed = 1;

        public static final int feedMotorId = 14;
        public static final int flyMotorId = 15;
    }

    private MotorIO feed;
    private MotorIO fly;

    public Shooter(MotorIO feedIO, MotorIO flyIO) {
        this.feed = feedIO;
        this.fly = flyIO;

        fly.connectInternalSensor(Constants.flyGearRatio);
    }

    public void setLocked(boolean locked) {
        feed.setBraking(locked);
        fly.setBraking(locked);
    }

    public void setDisabled(boolean disabled) {
        feed.setDisabled(disabled);
        fly.setDisabled(disabled);
    }

    public void setFlySpeed(double speed) {
        fly.setDutyCycle(speed);
    }

    public void flyShoot() {
        setFlySpeed(Constants.flySpeed);
    }

    public void flyReverse() {
        setFlySpeed(-Constants.flySpeed);
    }

    public void flyStop() {
        setFlySpeed(0);
    }

    public void setFeedSpeed(double speed) {
        feed.setDutyCycle(speed);
    }

    public void feedShoot() {
        setFeedSpeed(Constants.feedSpeed);
    }

    public void feedReverse() {
        setFeedSpeed(-Constants.feedSpeed);
    }

    public void feedStop() {
        setFeedSpeed(0);
    }

    @Override
    public void periodic() {
        setLocked(Constants.shooterLocked.get());
        setDisabled(Constants.shooterDisabled.get());

        fly.update();
        feed.update();
    }
}
