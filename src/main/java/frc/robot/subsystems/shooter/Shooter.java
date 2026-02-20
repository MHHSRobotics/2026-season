package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import frc.robot.io.MotorIO;

public class Shooter extends SubsystemBase {
    public static class Constants {
        // Subsystem Constants
        public static final LoggedNetworkBoolean shooterLocked = new LoggedNetworkBoolean("Shooter/Locked", true);
        public static final LoggedNetworkBoolean shooterDisabled = new LoggedNetworkBoolean("Shooter/Disabled", false);

        public static final double flySpeed = 400; // Target velocity in rad/s
        public static final double feedSpeed = 0.5;

        public static final boolean flyInverted = false;
        public static final boolean feedInverted = false;

        public static final int flyMotorId = 20;
        public static final int feedMotorId = 21;

        public static final double feedRatio = 1;
        public static final double flyRatio = 1;

        // Flywheel speed tolerance as a relative factor of speed
        public static final double tol = 0.1;

        // Used for simulation only
        public static final double feedInertia = 0.000066; // Inertia of feed wheels in kg m^2
        public static final double flyInertia = 0.005; // Inertia of fly wheels in kg m^2
    }

    private MotorIO feed;
    private MotorIO fly;

    private BangBangController controller;

    private double targetSpeed;

    public Shooter(MotorIO feedIO, MotorIO flyIO) {
        this.feed = feedIO;
        this.fly = flyIO;

        fly.setInverted(Constants.flyInverted);
        fly.connectInternalSensor(Constants.flyRatio);
        feed.setInverted(Constants.feedInverted);
        feed.connectInternalSensor(Constants.feedRatio);

        controller = new BangBangController();
    }

    public double getFlyVelocity() {
        return fly.getInputs().velocity;
    }

    private void setLocked(boolean locked) {
        feed.setBraking(locked);
        fly.setBraking(locked);
    }

    private void setDisabled(boolean disabled) {
        feed.setDisabled(disabled);
        fly.setDisabled(disabled);
    }

    public void setFlyTargetSpeed(double speed) {
        targetSpeed = speed;
    }

    public boolean atTargetSpeed() {
        return Math.abs(getFlyVelocity() - targetSpeed) / targetSpeed < Constants.tol;
    }

    public void flyShoot() {
        setFlyTargetSpeed(Constants.flySpeed);
    }

    public void flyStop() {
        setFlyTargetSpeed(0);
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

        fly.setDutyCycle(controller.calculate(getFlyVelocity(), targetSpeed));
        fly.update();
        feed.update();

        Logger.recordOutput("Shooter/TargetSpeed", targetSpeed);
        Logger.recordOutput("Shooter/AtTarget", atTargetSpeed());
    }
}
