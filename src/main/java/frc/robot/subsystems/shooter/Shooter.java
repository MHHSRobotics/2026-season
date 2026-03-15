package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.io.MotorIO;

public class Shooter extends SubsystemBase {
    public static class Constants {
        // Subsystem Constants
        public static final LoggedNetworkBoolean shooterLocked = new LoggedNetworkBoolean("Shooter/Locked", true);
        public static final LoggedNetworkBoolean shooterDisabled = new LoggedNetworkBoolean("Shooter/Disabled", false);

        public static final LoggedNetworkNumber defaultSpeed =
                new LoggedNetworkNumber("Shooter/DefaultSpeed", 500); // Default shooter speed

        public static final LoggedNetworkNumber flykP = new LoggedNetworkNumber("Shooter/FlykP", 1); // kP
        public static final LoggedNetworkNumber flykD = new LoggedNetworkNumber("Shooter/FlykD", 0.03); // kD

        public static final double feedSpeed = 0.5;

        public static final boolean flyInverted = false;
        public static final boolean flyInverted2 = true;
        public static final boolean feedInverted = false;

        public static final int flyMotorId = 16;
        public static final int flyMotorId2 = 17;
        public static final int feedMotorId = 18;

        public static final double feedRatio = 1;
        public static final double flyRatio = 1;

        // Flywheel speed tolerance as a relative factor of speed
        public static final double tol = 0.05;

        // Approximate exit angle of the fuel, used for accounting for movement
        public static final double exitAngle = Units.degreesToRadians(58);
        // Exit velocity/flywheel circumference speed
        public static final double efficiency = 0.4475;
        // Radius of flywheel (m)
        public static final double flywheelRadius = 0.05;
        // Location of shooter wrt center of bot
        public static final Translation2d shooterPos = new Translation2d(-0.2, 0);

        // Approx latency (s)
        public static final double latency = 0.05;

        // Used for simulation only
        public static final double feedInertia = 0.000066; // Inertia of feed wheels in kg m^2
        public static final double flyInertia = 0.005; // Inertia of fly wheels in kg m^2
    }

    private final MotorIO feed;
    private final MotorIO fly;
    private final MotorIO fly2;

    private double targetSpeed;

    public Shooter(MotorIO feedIO, MotorIO flyIO, MotorIO flyIO2) {
        feed = feedIO;
        fly = flyIO;
        fly2 = flyIO2;

        fly.setInverted(Constants.flyInverted);
        fly.connectInternalSensor(Constants.flyRatio);
        fly2.follow(Constants.flyMotorId, Constants.flyInverted ^ Constants.flyInverted2);
        feed.setInverted(Constants.feedInverted);
        feed.connectInternalSensor(Constants.feedRatio);
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
        if (Math.abs(targetSpeed) < 1e-6) {
            return false;
        }
        return Math.abs(getFlyVelocity() - targetSpeed) / Math.abs(targetSpeed) < Constants.tol;
    }

    public void defaultShoot() {
        setFlyTargetSpeed(Constants.defaultSpeed.get());
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
        if (targetSpeed == 0) {
            fly.coast();
        } else {
            fly.setVelocityWithCurrent(targetSpeed);
        }
        fly.setkP(Constants.flykP.get());
        fly.setkD(Constants.flykD.get());

        fly.update();
        fly2.update();
        feed.update();

        Logger.recordOutput("Shooter/TargetSpeed", targetSpeed);
        Logger.recordOutput("Shooter/AtTarget", atTargetSpeed());
    }
}
