package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.signals.GravityTypeValue;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

// Make the wrist subsystem move a single-jointed wrist to targets and show helpful
// visuals for students tuning it. All angles are wrist mechanism angles (radians).
public class Wrist extends SubsystemBase {

    public static class Constants {
        // All angles in this class are wrist mechanism angles (radians)

        // CAN device ID for the wrist motor controller
        public static final int motorId = 23;
        // Angle offset (radians) to line up the absolute encoder zero with the real wrist zero
        public static final double offset = 1.3;
        // Whether to flip motor direction (true means reverse forward/backward)
        public static final boolean motorInverted = false;

        // CAN device ID for the absolute encoder
        public static final int encoderId = 28;
        // Whether to flip encoder direction to match the wrist positive direction
        public static final boolean encoderInverted = false;

        public static final double gearRatio = 16.0; // Ratio of motor rotations to wrist rotations (unitless)
        public static final double encoderRatio =
                1.0; // Ratio of encoder rotations to wrist rotations (unitless, negative since encoder was inverted
        // before)

        public static final LoggedNetworkNumber kP =
                new LoggedNetworkNumber("Wrist/kP", 60); // (volts per radian) more voltage when farther from target
        public static final LoggedNetworkNumber kD =
                new LoggedNetworkNumber("Wrist/kD", 30); // (volts per rad/s) reacts to how fast error is changing
        public static final LoggedNetworkNumber kI =
                new LoggedNetworkNumber("Wrist/kI", 0); // (volts per rad) removes steady state error

        public static final LoggedNetworkNumber kS = new LoggedNetworkNumber(
                "Wrist/kS", 0.0); // (volts) voltage to get wrist moving (overcome static friction)
        public static final LoggedNetworkNumber kG = new LoggedNetworkNumber(
                "Wrist/kG", 25.0); // (volts) voltage to hold the wrist level (compensate gravity at 0 rad)
        public static final LoggedNetworkNumber kV = new LoggedNetworkNumber(
                "Wrist/kV", 0); // (volts per rad/s) voltage that scales with speed to overcome friction
        public static final LoggedNetworkNumber kA =
                new LoggedNetworkNumber("Wrist/kA", 0); // (volts per rad/s^2) extra voltage to help with acceleration

        public static final LoggedNetworkNumber maxVelocity = new LoggedNetworkNumber(
                "Wrist/maxVelocity", 6.3); // (rad/s) Motion Magic max speed for moving to a target
        public static final LoggedNetworkNumber maxAccel = new LoggedNetworkNumber(
                "Wrist/maxAccel", 6.3); // (rad/s^2) Motion Magic max acceleration for moving to a target

        public static final double statorCurrentLimit = 70; // (amps) limit on motor torque output
        public static final double supplyCurrentLimit = 60; // (amps) normal current limit pulled from battery
        public static final double supplyCurrentLowerLimit = 40; // (amps) reduce to this if over limit for some time
        public static final double supplyCurrentLowerTime = 0.3; // (seconds) time before lowering current limit

        public static final double momentOfInertia = 0.5219; // (kg·m^2) how hard it is to rotate the wrist
        public static final double wristLength = 0.1524; // Wrist length (meters)

        public static final double minAngle = Units.degreesToRadians(-135); // (radians) soft lower limit (~-135°)
        public static final double maxAngle = Units.degreesToRadians(90); // (radians) soft upper limit (~90°)
        public static final double startAngle = Units.degreesToRadians(0); // (radians) start angle in sim (~0°)

        // Wrist tolerance for considering target reached
        public static final double wristTolerance =
                Units.degreesToRadians(10); // (radians) how close we need to be to target

        public static final double rotorToSensorRatio =
                gearRatio / encoderRatio; // Ratio of motor rotations to encoder rotations (unitless)

        public static final LoggedNetworkBoolean wristLocked =
                new LoggedNetworkBoolean("Wrist/Locked", true); // Toggle to enable braking when stopped

        public static final LoggedNetworkBoolean wristDisabled =
                new LoggedNetworkBoolean("Wrist/Disabled", false); // Toggle to completely disable the wrist subsystem

        // Danger zones where wrist might hit the arm
        public static final double armDangerMin =
                Units.degreesToRadians(-110); // (radians) arm angle where wrist collision starts
        public static final double armDangerMax =
                Units.degreesToRadians(-45); // (radians) arm angle where wrist collision ends
    }

    // Wrist motor interface; handles real robot and simulation for us
    private MotorIO motor;

    // Absolute encoder interface; also supports simulation automatically
    private EncoderIO encoder;

    // Arm motor for wrist gravity feedforward
    private MotorIO armMotor;

    // On-screen drawing of the wrist for dashboards (length is visual only)
    private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);

    // The fixed base point for the wrist drawing
    private final LoggedMechanismRoot2d root = mech.getRoot("WristRoot", 1, 1.5);

    // The live wrist drawing that rotates to match the wrist angle (radians)
    private final LoggedMechanismLigament2d wrist =
            root.append(new LoggedMechanismLigament2d("Wrist", 0.5, 0, 6, new Color8Bit(Color.kOrange)));

    // Drawing that shows the wrist's target angle (radians)
    private final LoggedMechanismLigament2d goalWrist =
            root.append(new LoggedMechanismLigament2d("GoalWrist", 0.5, 0, 6, new Color8Bit(Color.kYellow)));

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

    public Wrist(MotorIO motorIO, EncoderIO encoderIO, MotorIO armMotor) {
        encoder = encoderIO;
        // Tell the encoder which direction is positive and the gear ratio to the wrist
        encoder.setInverted(Constants.encoderInverted);
        encoder.setGearRatio(Constants.encoderRatio);

        motor = motorIO;

        this.armMotor = armMotor;

        // Tell the motor which direction is forward (true = invert)
        motor.setInverted(Constants.motorInverted);
        // Tell the motor which encoder to use and how motor/encoder/wrist relate (ratios are unitless)
        motor.connectEncoder(encoderIO, Constants.rotorToSensorRatio, true);

        // Make the motor use cosine gravity compensation (more help when the wrist is level)
        motor.setFeedforwardType(GravityTypeValue.Arm_Cosine);

        motor.setOffset(Constants.offset);

        // Set limits on the motor
        motor.setLimits(Constants.minAngle, Constants.maxAngle);

        // Add the middle dot to the visualization
        root.append(new LoggedMechanismLigament2d("Middle", 0.0, 0, 10, new Color8Bit(Color.kPurple)));
    }

    // Tell the wrist motor how fast to spin (percent [-1 to 1], -1 = full backward, 1 = full forward)
    public void setSpeed(double value) {
        motor.setDutyCycle(value);
    }

    // Find out the current wrist angle (radians)
    public double getPosition() {
        return motor.getInputs().position;
    }

    // Find out how fast the wrist is rotating (radians per second)
    public double getVelocity() {
        return motor.getInputs().velocity;
    }

    // Tell the wrist to go to a target angle (radians). Example: 0 rad ≈ wrist straight forward.
    // We clamp to safe limits so the wrist won't try to drive past its allowed range.
    public void setGoal(double angle) {
        motor.setGoalWithCurrentMagic(
                MathUtil.clamp(angle, Constants.minAngle, Constants.maxAngle),
                () -> Constants.kG.get() * Math.cos(getPosition() + armMotor.getInputs().position));
    }

    // Find out the current target angle (radians)
    public double getGoal() {
        return motor.getInputs().setpoint;
    }

    // Find out if the wrist is close enough to its target (within tolerance)
    public boolean atGoal() {
        return Math.abs(getPosition() - getGoal()) < Constants.wristTolerance;
    }

    @Override
    public void periodic() {
        // This runs every robot loop (about 50 times per second) to update sensors,
        // show visuals, apply tuning numbers, and check for problems

        // Set braking based on user input
        motor.setBraking(Constants.wristLocked.get());

        // Disable the motor based on user input
        motor.setDisabled(Constants.wristDisabled.get());

        // 1) Update sensor/motor inputs so the latest values are available (logging and alerts happen automatically)
        motor.update();
        encoder.update();

        // 2) Update the on-screen wrist drawing to match the current wrist angle (radians)
        wrist.setAngle(Rotation2d.fromRadians(motor.getInputs().position));

        if (motor.getInputs().controlMode.startsWith("MM_")) {
            // If the motor is using Motion Magic (PID to a target), show the target and P/I/D/FF bars
            goalWrist.setLineWeight(6);
            pAmount.setLineWeight(6);
            dAmount.setLineWeight(6);
            fAmount.setLineWeight(6);
            iAmount.setLineWeight(6);

            // Set the target angle and how big each control term is (scaled down for drawing)
            goalWrist.setAngle(Rotation2d.fromRadians(motor.getInputs().setpoint));
            pAmount.setLength(motor.getInputs().propOutput / 100);
            dAmount.setLength(motor.getInputs().derivOutput / 100);
            fAmount.setLength(motor.getInputs().feedforward / 100);
            iAmount.setLength(motor.getInputs().intOutput / 100);
        } else {
            // Hide the target and P/I/D/FF bars when not using Motion Magic
            goalWrist.setLineWeight(0);
            pAmount.setLineWeight(0);
            dAmount.setLineWeight(0);
            fAmount.setLineWeight(0);
            iAmount.setLineWeight(0);
        }

        // 3) Send the mechanism drawing to the logs/dashboard
        Logger.recordOutput("Wrist/Visualization", mech);

        // 4) Read tuning numbers and apply them to the motor controller (units noted above)
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
