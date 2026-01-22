package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

public class Shooter extends SubsystemBase {
    public static class Constants {
        // feedConstants
        public static final boolean feedBraking = false;
        public static final double feedGearRatio = 0.0;
        public static final double feedStatorCurrentLimit = 0.0;
        public static final double feedOffset = 0.0;

        public static final double feedVelocityWithVoltage = 0.0;

        // flyConstants
        public static final boolean flyBraking = false;
        public static final double flyGearRatio = 0.0;
        public static final double flyStatorCurrentLimit = 0.0;
        public static final double flyOffset = 0.0;

        public static final double flyVelocityWithVoltage = 0.0;

        // pitchConstants
        public static final boolean pitchBraking = false;
        public static final double pitchGearRatio = 0.0;
        public static final double pitchStatorCurrentLimit = 0.0;
        public static final double pitchToSensorRatio = 0.0;
        public static final double pitchOffset = 0.0;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kG = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
    }

    private MotorIO feed;
    private MotorIO fly;
    private MotorIO pitch;
    private EncoderIO pitchEncoder;

    public Shooter(MotorIO feedIO, MotorIO flyIO, MotorIO pitchIO, EncoderIO pitchEncoderIO) {
        this.feed = feedIO;
        this.fly = flyIO;
        this.pitch = pitchIO;
        this.pitchEncoder = pitchEncoderIO;

        feed.setBraking(Constants.feedBraking);
        feed.setGearRatio(Constants.feedGearRatio);
        feed.setStatorCurrentLimit(Constants.feedStatorCurrentLimit);
        feed.setOffset(Constants.feedOffset);

        fly.setBraking(Constants.flyBraking);
        fly.setGearRatio(Constants.flyGearRatio);
        fly.setStatorCurrentLimit(Constants.flyStatorCurrentLimit);
        fly.setOffset(Constants.flyOffset);

        pitch.setBraking(Constants.pitchBraking);
        pitch.setGearRatio(Constants.pitchGearRatio);
        pitch.setStatorCurrentLimit(Constants.pitchStatorCurrentLimit);
        pitch.connectEncoder(pitchEncoder, Constants.pitchToSensorRatio);
        pitch.setOffset(Constants.pitchOffset);
        pitch.setkP(Constants.kP);
        pitch.setkI(Constants.kI);
        pitch.setkD(Constants.kD);
        pitch.setkG(Constants.kG);
        pitch.setkS(Constants.kS);
        pitch.setkV(Constants.kV);
        pitch.setkA(Constants.kA);
    }

    public void setLocked(boolean locked) {
        feed.setBraking(locked);
        fly.setBraking(locked);
        pitch.setBraking(locked);
    }

    public void setDisabled(boolean disabled) {
        feed.setBraking(disabled);
        fly.setBraking(disabled);
        pitch.setBraking(disabled);
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

    public double pitchGetPosition() {
        return pitch.getInputs().position;
    }

    public double pitchGetGoal() {
        return pitch.getInputs().setpoint;
    }

    public void pitchSetGoal(double position) {
        pitch.setGoalWithVoltage(position);
    }

    public void pitchStop() {
        pitch.setVoltage(0);
    }
}
