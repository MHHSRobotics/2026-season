package frc.robot.subsystems.arm;

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

// Make the arm subsystem move a single-jointed arm to targets and show helpful
// visuals for students tuning it. All angles are arm mechanism angles (radians).
public class Arm extends SubsystemBase {

    public static class Constants {
        // All angles in this class are arm mechanism angles (radians)

        // CAN device ID for the arm motor controller
        public static final int motorId = 22;
        // Angle offset (radians) to line up the absolute encoder zero with the real arm zero
        public static final double offset = 2.45; // -2.85;
        // Whether to flip motor direction (true means reverse forward/backward)
        public static final boolean motorInverted = false;

        // CAN device ID for the absolute encoder
        public static final int encoderId = 26;
        // Whether to flip encoder direction to match the arm positive direction
        public static final boolean encoderInverted = true;

        public static final double gearRatio = 175 / 2.; // Ratio of motor rotations to arm rotations (unitless)
        public static final double encoderRatio = 1; // Ratio of encoder rotations to arm rotations (unitless)

        public static final double armTolerance = Units.degreesToRadians(5);

        public static final LoggedNetworkNumber kP =
                new LoggedNetworkNumber("Arm/kP", 50); // (volts per radian) more voltage when farther from target
        public static final LoggedNetworkNumber kD =
                new LoggedNetworkNumber("Arm/kD", 20); // (volts per rad/s) reacts to how fast error is changing

        public static final LoggedNetworkNumber kS =
                new LoggedNetworkNumber("Arm/kS", 0); // (volts) voltage to get arm moving (overcome static friction)
        public static final LoggedNetworkNumber kG = new LoggedNetworkNumber(
                "Arm/kG", 12); // (volts) voltage to hold the arm level (compensate gravity at 0 rad)
        public static final LoggedNetworkNumber kV = new LoggedNetworkNumber(
                "Arm/kV", 0); // (volts per rad/s) voltage that scales with speed to overcome friction
        public static final LoggedNetworkNumber kA =
                new LoggedNetworkNumber("Arm/kA", 0); // (volts per rad/s^2) extra voltage to help with acceleration
        public static final LoggedNetworkNumber kI = new LoggedNetworkNumber("Arm/kI", 0);

        public static final LoggedNetworkNumber maxVelocity =
                new LoggedNetworkNumber("Arm/maxVelocity", 10); // (rad/s) Motion Magic max speed for moving to a target
        public static final LoggedNetworkNumber maxAccel = new LoggedNetworkNumber(
                "Arm/maxAccel", 4); // (rad/s^2) Motion Magic max acceleration for moving to a target

        public static final double statorCurrentLimit = 70; // (amps) limit on motor torque output
        public static final double supplyCurrentLimit = 60; // (amps) normal current limit pulled from battery
        public static final double supplyCurrentLowerLimit = 40; // (amps) reduce to this if over limit for some time
        public static final double supplyCurrentLowerTime = 0.3; // (seconds) time before lowering current limit

        public static final double mass = 10; // (kg) estimated arm mass for simulation
        public static final double armLength = 0.6; // Arm length (meters)
        public static final double moi =
                (1. / 3.) * mass * armLength * armLength; // (kg·m^2) how hard it is to rotate the arm

        public static final double minAngle = Units.degreesToRadians(-45); // (radians) soft lower limit (~-45°)
        public static final double maxAngle = Units.degreesToRadians(140); // (radians) soft upper limit (~140°)
        public static final double startAngle = Units.degreesToRadians(90); // (radians) start angle in sim (~90°)

        public static final double rotorToSensorRatio =
                gearRatio / encoderRatio; // Ratio of motor rotations to encoder rotations (unitless)

        public static final LoggedNetworkBoolean armLocked =
                new LoggedNetworkBoolean("Arm/Locked", true); // Toggle to enable braking when stopped

        public static final LoggedNetworkBoolean armDisabled =
                new LoggedNetworkBoolean("Arm/Disabled", false); // Toggle to completely disable the arm subsystem

        // Angle constants
        public static final double defaultAngle = Units.degreesToRadians(90);
    }

    // Arm motor interface; handles real robot and simulation for us
    private MotorIO motor;

    // Absolute encoder interface; also supports simulation automatically
    private EncoderIO encoder;

    // On-screen drawing of the arm for dashboards (length is visual only)
    private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);

    // The fixed base point for the arm drawing
    private final LoggedMechanismRoot2d root = mech.getRoot("ArmRoot", 1, 1.5);

    // The live arm drawing that rotates to match the arm angle (radians)
    private final LoggedMechanismLigament2d arm =
            root.append(new LoggedMechanismLigament2d("Arm", 1.0, 0, 6, new Color8Bit(Color.kRed)));

    // Drawing that shows the arm's target angle (radians)
    private final LoggedMechanismLigament2d goalArm =
            root.append(new LoggedMechanismLigament2d("GoalArm", 1.0, 0, 6, new Color8Bit(Color.kYellow)));

    // Base point for the proportional (P) bar visualization
    private final LoggedMechanismRoot2d pRoot = mech.getRoot("PRoot", 2.5, 2);

    // Base point for the derivative (D) bar visualization
    private final LoggedMechanismRoot2d dRoot = mech.getRoot("DRoot", 2.6, 2);

    // Base point for the feedforward (FF) bar visualization
    private final LoggedMechanismRoot2d fRoot = mech.getRoot("FRoot", 2.7, 2);

    // Base point for the feedforward (FF) bar visualization
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

    // Integral (FF) amount bar
    private final LoggedMechanismLigament2d iAmount =
            iRoot.append(new LoggedMechanismLigament2d("IAmount", 1.0, 90, 6, new Color8Bit(Color.kRed)));

    public Arm(MotorIO motorIO, EncoderIO encoderIO) {
        encoder = encoderIO;
        // Tell the encoder which direction is positive and the gear ratio to the arm
        encoder.setInverted(Constants.encoderInverted);
        encoder.setGearRatio(Constants.encoderRatio);

        motor = motorIO;

        // Tell the motor which direction is forward (true = invert)
        motor.setInverted(Constants.motorInverted);
        // Tell the motor which encoder to use and how motor/encoder/arm relate (ratios are unitless)
        motor.connectEncoder(encoderIO, Constants.rotorToSensorRatio, false);

        // Make the motor use cosine gravity compensation (more help when the arm is level)
        motor.setFeedforwardType(GravityTypeValue.Arm_Cosine);

        // Set motor offset
        motor.setOffset(Constants.offset);

        motor.setLimits(Constants.minAngle, Constants.maxAngle);

        // Add middle dot to visualization
        root.append(new LoggedMechanismLigament2d("Middle", 0.0, 0, 10, new Color8Bit(Color.kBlue)));
    }

    // Tell the arm motor how fast to spin (percent [-1 to 1], -1 = full backward, 1 = full forward)
    public void setSpeed(double value) {
        motor.setDutyCycle(value);
    }

    public double getPosition() {
        return motor.getInputs().position;
    }

    public double getVelocity() {
        return motor.getInputs().velocity;
    }

    // Tell the arm to go to a target angle (radians). Example: 0 rad ≈ arm straight forward.
    // We clamp to safe limits so the arm won't try to drive past its allowed range.
    public void setGoal(double pos) {
        motor.setGoalWithCurrentMagic(MathUtil.clamp(pos, Constants.minAngle, Constants.maxAngle));
    }

    // Find out the current target angle (radians)
    public double getGoal() {
        return motor.getInputs().setpoint;
    }

    // Whether the error is within tolerance
    public boolean atGoal() {
        return Math.abs(getPosition() - getGoal()) < Constants.armTolerance;
    }

    @Override
    public void periodic() {
        // This runs every robot loop (about 50 times per second) to update sensors,
        // show visuals, apply tuning numbers, and check for problems

        // Set braking based on user input
        motor.setBraking(Constants.armLocked.get());

        // Disable the motor based on user input
        motor.setDisabled(Constants.armDisabled.get());

        // 1) Update sensor/motor inputs so the latest values are available (logging and alerts happen automatically)
        motor.update();
        encoder.update();

        // 2) Update the on-screen arm drawing to match the current arm angle (radians)
        arm.setAngle(Rotation2d.fromRadians(motor.getInputs().position));

        if (motor.getInputs().controlMode.startsWith("MM_")) {
            // If the motor is using Motion Magic (PID to a target), show the target and P/D/FF bars
            goalArm.setLineWeight(6);
            pAmount.setLineWeight(6);
            dAmount.setLineWeight(6);
            fAmount.setLineWeight(6);
            iAmount.setLineWeight(6);

            // Set the target angle and how big each control term is (scaled down for drawing)
            goalArm.setAngle(Rotation2d.fromRadians(motor.getInputs().setpoint));
            pAmount.setLength(motor.getInputs().propOutput / 100);
            dAmount.setLength(motor.getInputs().derivOutput / 100);
            fAmount.setLength(motor.getInputs().feedforward / 100);
            iAmount.setLength(motor.getInputs().intOutput / 100);
        } else {
            // Hide the target and P/D/FF bars when not using Motion Magic
            goalArm.setLineWeight(0);
            pAmount.setLineWeight(0);
            dAmount.setLineWeight(0);
            fAmount.setLineWeight(0);
            iAmount.setLineWeight(0);
        }

        // 3) Send the mechanism drawing to the logs/dashboard
        Logger.recordOutput("Arm/Visualization", mech);

        // 4) Read tuning numbers and apply them to the motor controller (units noted above)
        motor.setkP(Constants.kP.get());
        motor.setkD(Constants.kD.get());
        motor.setkG(Constants.kG.get());
        motor.setkS(Constants.kS.get());
        motor.setkV(Constants.kV.get());
        motor.setkA(Constants.kA.get());
        motor.setkI(Constants.kI.get());
        motor.setMaxVelocity(Constants.maxVelocity.get());
        motor.setMaxAccel(Constants.maxAccel.get());
    }
}
