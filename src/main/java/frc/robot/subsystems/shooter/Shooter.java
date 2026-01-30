package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.signals.GravityTypeValue;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

public class Shooter extends SubsystemBase {
    public static class Constants {
        // Subsystem Constants
        public static final LoggedNetworkBoolean shooterLocked = new LoggedNetworkBoolean("Shooter/locked", true);
        public static final LoggedNetworkBoolean shooterDisabled = new LoggedNetworkBoolean("Shooter/disabled", true);

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

        // pitchEncoderConstants
        public static final boolean pitchEncoderInverted = false;
        public static final double pitchEncoderGearRatio = 0.0;

        // pitchConstants
        public static final boolean pitchBraking = false;
        public static final GravityTypeValue pitchFeedForward = GravityTypeValue.Arm_Cosine;
        public static final double pitchGearRatio = 0.0;
        public static final double pitchStatorCurrentLimit = 0.0;
        public static final double pitchToSensorRatio = 0.0;
        public static final double pitchOffset = 0.0;

        public static final LoggedNetworkNumber pitchMaxAccel = new LoggedNetworkNumber("Pitch/maxAccel", 0.0);
        public static final LoggedNetworkNumber pitchMaxVelocity = new LoggedNetworkNumber("Pitch/maxVelocity", 0.0);

        public static final LoggedNetworkNumber kP = new LoggedNetworkNumber("Pitch/kP", 0.0);
        public static final LoggedNetworkNumber kI = new LoggedNetworkNumber("Pitch/kI", 0.0);
        public static final LoggedNetworkNumber kD = new LoggedNetworkNumber("Pitch/kD", 0.0);
        public static final LoggedNetworkNumber kG = new LoggedNetworkNumber("Pitch/kG", 0.0);
        public static final LoggedNetworkNumber kS = new LoggedNetworkNumber("Pitch/kS", 0.0);
        public static final LoggedNetworkNumber kV = new LoggedNetworkNumber("Pitch/kV", 0.0);
        public static final LoggedNetworkNumber kA = new LoggedNetworkNumber("Pitch/kA", 0.0);
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

        pitchEncoder.setInverted(Constants.pitchEncoderInverted);
        pitchEncoder.setGearRatio(Constants.pitchEncoderGearRatio);

        feed.setBraking(Constants.feedBraking);
        feed.setGearRatio(Constants.feedGearRatio);
        feed.setStatorCurrentLimit(Constants.feedStatorCurrentLimit);
        feed.setOffset(Constants.feedOffset);

        fly.setBraking(Constants.flyBraking);
        fly.setGearRatio(Constants.flyGearRatio);
        fly.setStatorCurrentLimit(Constants.flyStatorCurrentLimit);
        fly.setOffset(Constants.flyOffset);

        pitch.setBraking(Constants.pitchBraking);
        pitch.setFeedforwardType(Constants.pitchFeedForward);
        pitch.setGearRatio(Constants.pitchGearRatio);
        pitch.setStatorCurrentLimit(Constants.pitchStatorCurrentLimit);
        pitch.connectEncoder(pitchEncoder, Constants.pitchToSensorRatio);
        pitch.setOffset(Constants.pitchOffset);
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

    @Override
    public void periodic() {
        setLocked(Constants.shooterLocked.get());
        setDisabled(Constants.shooterDisabled.get());

        pitch.setkP(Constants.kP.get());
        pitch.setkI(Constants.kI.get());
        pitch.setkD(Constants.kD.get());
        pitch.setkG(Constants.kG.get());
        pitch.setkS(Constants.kS.get());
        pitch.setkV(Constants.kV.get());
        pitch.setkA(Constants.kA.get());
        pitch.setMaxAccel(Constants.pitchMaxAccel.get());
        pitch.setMaxVelocity(Constants.pitchMaxVelocity.get());
    }
}
