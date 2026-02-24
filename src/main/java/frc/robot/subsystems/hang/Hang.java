package frc.robot.subsystems.hang;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

public class Hang extends SubsystemBase {
    public static class Constants {
        public static final int motorId = 25;

        public static final int encoderId = 26;

        public static final boolean motorInverted = false;
        public static final boolean encoderInverted=false;

        public static final double encoderRatio=1;

        public static final double motorRatio=32;
        public static final double offset=0;

        public static final double minAngle=Units.degreesToRadians(-100);
        public static final double maxAngle=Units.degreesToRadians(100);
        public static final double startAngle=Units.degreesToRadians(-100); // start angle in sim

        public static final LoggedNetworkBoolean hangLocked =
                new LoggedNetworkBoolean("Hang/Locked", true); // Toggle to enable braking when stopped

        public static final LoggedNetworkBoolean hangDisabled =
                new LoggedNetworkBoolean("Hang/Disabled", false); // Toggle to completely disable the hang subsystem

        public static final LoggedNetworkNumber kP =
                new LoggedNetworkNumber("Hang/kP", 0); // (volts per radian) more voltage when farther from target
        public static final LoggedNetworkNumber kD =
                new LoggedNetworkNumber("Hang/kD", 0); // (volts per rad/s) reacts to how fast error is changing
        public static final LoggedNetworkNumber kI =
                new LoggedNetworkNumber("Hang/kI", 0); // (volts per rad) removes steady state error

        public static final LoggedNetworkNumber kS = new LoggedNetworkNumber(
                "Hang/kS", 0); // (volts) voltage to get wrist moving (overcome static friction)
        public static final LoggedNetworkNumber kG = new LoggedNetworkNumber(
                "Hang/kG", 0); // (volts) voltage to hold the wrist level (compensate gravity at 0 rad)
        public static final LoggedNetworkNumber kV = new LoggedNetworkNumber(
                "Hang/kV", 0); // (volts per rad/s) voltage that scales with speed to overcome friction
        public static final LoggedNetworkNumber kA =
                new LoggedNetworkNumber("Hang/kA", 0); // (volts per rad/s^2) extra voltage to help with acceleration

        public static final LoggedNetworkNumber maxVelocity = new LoggedNetworkNumber(
                "Hang/maxVelocity", 6.3); // (rad/s) Motion Magic max speed for moving to a target
        public static final LoggedNetworkNumber maxAccel = new LoggedNetworkNumber(
                "Hang/maxAccel", 6.3); // (rad/s^2) Motion Magic max acceleration for moving to a target

        public static final LoggedNetworkNumber verticalPos=new LoggedNetworkNumber("Hang/VerticalPos",1.57);
    }

    private MotorIO motor;
    private EncoderIO encoder;

    // On-screen drawing of the wrist for dashboards (length is visual only)
    private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);

    // The fixed base point for the wrist drawing
    private final LoggedMechanismRoot2d root = mech.getRoot("HangRoot", 1, 1.5);

    // The live wrist drawing that rotates to match the wrist angle (radians)
    private final LoggedMechanismLigament2d hang =
            root.append(new LoggedMechanismLigament2d("Wrist", 0.5, 0, 6, new Color8Bit(Color.kOrange)));

    // Drawing that shows the wrist's target angle (radians)
    private final LoggedMechanismLigament2d goalHang =
            root.append(new LoggedMechanismLigament2d("GoalHang", 0.5, 0, 6, new Color8Bit(Color.kYellow)));

    // Base point for the proportional (P) bar visualization
    private final LoggedMechanismRoot2d pRoot = mech.getRoot("PRoot", 2.5, 2);

    // Base point for the derivative (D) bar visualization
    private final LoggedMechanismRoot2d dRoot = mech.getRoot("DRoot", 2.6, 2);

    // Base point for the feedforward (FF) bar visualization
    private final LoggedMechanismRoot2d fRoot = mech.getRoot("FRoot", 2.7, 2);

    // Base point for the integral bar visualization
    private final LoggedMechanismRoot2d iRoot = mech.getRoot("IRoot", 2.8, 2);

    // Proportional (P) amount bar
    private final LoggedMechanismLigament2d pAmount =
            pRoot.append(new LoggedMechanismLigament2d("PAmount", 1.0, 90, 6, new Color8Bit(Color.kBlue)));

    // Derivative (D) amount bar
    private final LoggedMechanismLigament2d dAmount =
            dRoot.append(new LoggedMechanismLigament2d("DAmount", 1.0, 90, 6, new Color8Bit(Color.kGreen)));

    // Feedforward (FF) amount bar
    private final LoggedMechanismLigament2d fAmount =
            fRoot.append(new LoggedMechanismLigament2d("FAmount", 1.0, 90, 6, new Color8Bit(Color.kWhite)));

    // Integral amount bar
    private final LoggedMechanismLigament2d iAmount =
            iRoot.append(new LoggedMechanismLigament2d("IAmount", 1.0, 90, 6, new Color8Bit(Color.kRed)));

    public Hang(MotorIO motorIO,EncoderIO encoderIO) {
        motor = motorIO;
        encoder=encoderIO;

        encoder.setInverted(Constants.encoderInverted);
        encoder.setGearRatio(Constants.encoderRatio);

        motor.setInverted(Constants.motorInverted);
        motor.connectEncoder(encoder, Constants.motorRatio,true);

        motor.setFeedforwardType(GravityTypeValue.Arm_Cosine);
        motor.setOffset(Constants.offset);

        motor.setLimits(Constants.minAngle, Constants.maxAngle);

        motor.setStaticFeedforwardType(StaticFeedforwardSignValue.UseClosedLoopSign);
        
    }

    public void setSpeed(double speed) {
        motor.setDutyCycle(speed);
    }

    public double getPosition(){
        return motor.getInputs().position;
    }

    public double getVelocity(){
        return motor.getInputs().velocity;
    }

    public void setHingeGoal(double goal) {
        motor.setGoalWithCurrentMagic(goal, () -> {
            double position = motor.getInputs().position;
            return Constants.kG.get() * Math.cos(position + Math.PI / 2 - Constants.verticalPos.get());
        });
    }

    public double getGoal(){
        return motor.getInputs().setpoint;
    }

    @Override
    public void periodic() {
        motor.update();
        encoder.update();

        motor.setBraking(Constants.hangLocked.get());

        motor.setDisabled(Constants.hangDisabled.get());

        hang.setAngle(Rotation2d.fromRadians(motor.getInputs().position));

        if (motor.getInputs().controlMode.startsWith("MM_")) {
            // If the motor is using Motion Magic (PID to a target), show the target and P/I/D/FF bars
            goalHang.setLineWeight(6);
            pAmount.setLineWeight(6);
            dAmount.setLineWeight(6);
            fAmount.setLineWeight(6);
            iAmount.setLineWeight(6);

            // Set the target angle and how big each control term is (scaled down for drawing)
            goalHang.setAngle(Rotation2d.fromRadians(motor.getInputs().setpoint));
            pAmount.setLength(motor.getInputs().propOutput / 100);
            dAmount.setLength(motor.getInputs().derivOutput / 100);
            fAmount.setLength(motor.getInputs().feedforward / 100);
            iAmount.setLength(motor.getInputs().intOutput / 100);
        } else {
            // Hide the target and P/I/D/FF bars when not using Motion Magic
            goalHang.setLineWeight(0);
            pAmount.setLineWeight(0);
            dAmount.setLineWeight(0);
            fAmount.setLineWeight(0);
            iAmount.setLineWeight(0);
        }

        Logger.recordOutput("Hang/Visualization",mech);

        motor.setkP(Constants.kP.get());
        motor.setkD(Constants.kD.get());
        motor.setkS(Constants.kS.get());
        motor.setkV(Constants.kV.get());
        motor.setkA(Constants.kA.get());
        motor.setkI(Constants.kI.get());
        motor.setMaxVelocity(Constants.maxVelocity.get());
        motor.setMaxAccel(Constants.maxAccel.get());
    }
}
