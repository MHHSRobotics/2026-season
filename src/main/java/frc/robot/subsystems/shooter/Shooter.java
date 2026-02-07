package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import frc.robot.io.MotorIO;

public class Shooter extends SubsystemBase {
    public static class Constants {
        // Subsystem Constants
        public static final LoggedNetworkBoolean shooterLocked = new LoggedNetworkBoolean("Shooter/Locked", false);
        public static final LoggedNetworkBoolean shooterDisabled = new LoggedNetworkBoolean("Shooter/Disabled", false);

        // feedConstants
        public static final double feedGearRatio = 0.0;

        public static final double feedVelocityWithVoltage = 0.0;

        // flyConstants
        public static final double flyGearRatio = 0.0;

        public static final double flyVelocityWithVoltage = 0.0;

        public static final int feedID = 14;
        public static final int flyID = 15;
    }

    private MotorIO feed;
    private MotorIO fly;

    public Shooter(MotorIO feedIO, MotorIO flyIO) {
        this.feed = feedIO;
        this.fly = flyIO;

        feed.setGearRatio(Constants.feedGearRatio);

        fly.setGearRatio(Constants.flyGearRatio);
    }

    public void setLocked(boolean locked) {
        feed.setBraking(locked);
        fly.setBraking(locked);
    }

    public void setDisabled(boolean disabled) {
        feed.setDisabled(disabled);
        fly.setDisabled(disabled);
    }

    public void feedShoot() {
        feed.setVelocityWithVoltage(Constants.feedVelocityWithVoltage);
    }

    public void feedStop() {
        feed.setVelocityWithVoltage(0);
    }

    public void flyShoot() {
        fly.setVelocityWithVoltage(Constants.flyVelocityWithVoltage);
    }

    public void flyStop() {
        fly.setVelocityWithVoltage(0);
    }

    @Override
    public void periodic() {
        setLocked(Constants.shooterLocked.get());
        setDisabled(Constants.shooterDisabled.get());

        fly.update();
        feed.update();
    }
}
