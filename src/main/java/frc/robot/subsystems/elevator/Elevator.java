package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
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

// Make the elevator subsystem move a 2-motor elevator to height targets and show helpful
// visuals for students tuning it. All heights are in meters.
public class Elevator extends SubsystemBase {

    public static class Constants {
        // All heights in this class are in meters

        // CAN device IDs for the elevator motor controllers
        public static final int leftMotorId = 20;
        public static final int rightMotorId = 21;

        // Height offset (meters) to line up the absolute encoder zero with the real elevator zero
        public static final double offset = 0;

        // Whether to flip motor directions (true means reverse forward/backward)
        public static final boolean leftMotorInverted = false;
        public static final boolean rightMotorInverted = true; // Right motor typically inverted to match left

        // CAN device ID for the absolute encoder
        public static final int encoderId = 27;
        // Whether to flip encoder direction to match the elevator positive direction
        public static final boolean encoderInverted = true;

        public static final double gearRatio = 8.0; // Ratio of motor rotations to drum rotations (unitless)
        public static final double encoderRatio = 0.5; // Ratio of encoder rotations to drum rotations (unitless)

        public static final double drumRadius = 0.022; // Ratio of meters to drum radians (meters)

        public static final LoggedNetworkNumber kP =
                new LoggedNetworkNumber("Elevator/kP", 80); // (volts per meter) more voltage when farther from target
        public static final LoggedNetworkNumber kI = new LoggedNetworkNumber(
                "Elevator/kI", 60); // (volts per meter-second) helps eliminate steady-state error
        public static final LoggedNetworkNumber kD =
                new LoggedNetworkNumber("Elevator/kD", 70); // (volts per m/s) reacts to how fast error is changing

        public static final LoggedNetworkNumber kS = new LoggedNetworkNumber(
                "Elevator/kS", 0); // (volts) voltage to get elevator moving (overcome static friction)
        public static final LoggedNetworkNumber kG = new LoggedNetworkNumber(
                "Elevator/kG", 14.31); // (volts) voltage to hold the elevator up (compensate gravity)
        public static final LoggedNetworkNumber kV = new LoggedNetworkNumber(
                "Elevator/kV", 0); // (volts per m/s) voltage that scales with speed to overcome friction
        public static final LoggedNetworkNumber kA =
                new LoggedNetworkNumber("Elevator/kA", 0); // (volts per m/s^2) extra voltage to help with acceleration

        public static final LoggedNetworkNumber maxVelocity = new LoggedNetworkNumber(
                "Elevator/maxVelocity", 3.5); // (m/s) Motion Magic max speed for moving to a target
        public static final LoggedNetworkNumber maxAccel = new LoggedNetworkNumber(
                "Elevator/maxAccel", 8); // (m/s^2) Motion Magic max acceleration for moving to a target

        public static final double statorCurrentLimit = 70; // (amps) limit on motor torque output
        public static final double supplyCurrentLimit = 60; // (amps) normal current limit pulled from battery
        public static final double supplyCurrentLowerLimit = 40; // (amps) reduce to this if over limit for some time
        public static final double supplyCurrentLowerTime = 0.3; // (seconds) time before lowering current limit

        // Elevator tolerance for considering target reached
        public static final double elevatorTolerance = 0.05; // (meters) how close we need to be to target

        // Simulation constants
        public static final double carriageMass = 13.0; // (kg) estimated elevator carriage mass for simulation
        public static final double minHeight = 0.1; // (meters) soft lower limit
        public static final double maxHeight = 1.12; // (meters) soft upper limit
        public static final double startHeight = 0.1; // (meters) start height in sim

        public static final double rotorToSensorRatio =
                gearRatio / encoderRatio; // Ratio of motor rotations to encoder rotations (unitless)

        public static final double sensorToMechanismRatio = encoderRatio / drumRadius; // Encoder radians per meter

        public static final LoggedNetworkBoolean elevatorLocked =
                new LoggedNetworkBoolean("Elevator/Locked", true); // Toggle to enable braking when stopped

        public static final LoggedNetworkBoolean elevatorDisabled = new LoggedNetworkBoolean(
                "Elevator/Disabled", false); // Toggle to completely disable the elevator subsystem
    }

    // Elevator motor interfaces; handles real robot and simulation for us
    private MotorIO leftMotor;
    private MotorIO rightMotor;

    // Absolute encoder interface; also supports simulation automatically
    private EncoderIO encoder;

    // On-screen drawing of the elevator for dashboards (height is visual only)
    private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);

    // The fixed base point for the elevator drawing
    private final LoggedMechanismRoot2d root = mech.getRoot("ElevatorRoot", 1, 0.1);

    // The live elevator drawing that moves up/down to match the elevator height (meters)
    private final LoggedMechanismLigament2d elevator =
            root.append(new LoggedMechanismLigament2d("Elevator", 1.0, 90, 6, new Color8Bit(Color.kBlue)));

    // The fixed base point for the elevator drawing
    private final LoggedMechanismRoot2d goalRoot = mech.getRoot("ElevatorGoalRoot", 1.1, 0.1);

    // Drawing that shows the elevator's target height (meters)
    private final LoggedMechanismLigament2d goalElevator =
            goalRoot.append(new LoggedMechanismLigament2d("GoalElevator", 1.0, 90, 6, new Color8Bit(Color.kYellow)));

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

    // Integral amount bar
    private final LoggedMechanismLigament2d iAmount =
            iRoot.append(new LoggedMechanismLigament2d("IAmount", 1.0, 90, 6, new Color8Bit(Color.kRed)));

    public Elevator(MotorIO leftMotorIO, MotorIO rightMotorIO, EncoderIO encoderIO) {
        encoder = encoderIO;
        // Tell the encoder which direction is positive and the gear ratio to the elevator
        encoder.setInverted(Constants.encoderInverted);
        encoder.setGearRatio(Constants.sensorToMechanismRatio);

        leftMotor = leftMotorIO;
        rightMotor = rightMotorIO;

        // Tell the motors which direction is forward (true = invert)
        leftMotor.setInverted(Constants.leftMotorInverted);
        rightMotor.setInverted(Constants.rightMotorInverted);

        // Tell the left motor which encoder to use and how motor/encoder/elevator relate (ratios are unitless)
        leftMotor.connectEncoder(encoderIO, Constants.rotorToSensorRatio);

        // Add offset to motor
        leftMotor.setOffset(Constants.offset);

        // Add limits to motor (needs to be done after offset)
        leftMotor.setLimits(Constants.minHeight, Constants.maxHeight);

        // Make the motors use elevator gravity compensation (constant help against gravity)
        leftMotor.setFeedforwardType(GravityTypeValue.Elevator_Static);

        // Make the right motor follow the left motor (they should move together)
        rightMotor.follow(
                Constants.leftMotorId,
                Constants.leftMotorInverted
                        ^ Constants
                                .rightMotorInverted); // take the XOR of the two inverts to calculate relative inversion

        // Add middle dot to visualization
        root.append(new LoggedMechanismLigament2d("Middle", 0.0, 0, 10, new Color8Bit(Color.kGreen)));
    }

    // Tell the elevator motors how fast to spin (percent [-1 to 1], -1 = full down, 1 = full up)
    public void setSpeed(double value) {
        leftMotor.setDutyCycle(value);
        // Right motor follows automatically, no need to set it separately
    }

    // Find out how high the elevator is right now (height in meters)
    public double getPosition() {
        return leftMotor.getInputs().position;
    }

    // Find out how fast the elevator is moving (speed in meters per second)
    public double getVelocity() {
        return leftMotor.getInputs().velocity;
    }

    // Tell the elevator to go to a target height (meters). Example: 0.5 = half meter up.
    // We clamp to safe limits so the elevator won't try to drive past its allowed range.
    public void setGoal(double height) {
        leftMotor.setGoalWithCurrentMagic(MathUtil.clamp(height, Constants.minHeight, Constants.maxHeight));
    }

    // Find out the current target height (meters)
    public double getGoal() {
        return leftMotor.getInputs().setpoint;
    }

    // Find out if the elevator is close enough to its target (within tolerance)
    public boolean atGoal() {
        return Math.abs(getPosition() - getGoal()) < Constants.elevatorTolerance;
    }

    @Override
    public void periodic() {
        // This runs every robot loop (about 50 times per second) to update sensors,
        // show visuals, apply tuning numbers, and check for problems

        // Set braking based on user input
        leftMotor.setBraking(Constants.elevatorLocked.get());
        rightMotor.setBraking(false);

        // Disable the motors based on user input
        leftMotor.setDisabled(Constants.elevatorDisabled.get());
        rightMotor.setDisabled(Constants.elevatorDisabled.get());

        // 1) Update sensor/motor inputs so the latest values are available (logging and alerts happen automatically)
        leftMotor.update();
        rightMotor.update();
        encoder.update();

        // 2) Update the on-screen elevator drawing to match the current elevator height (meters)
        elevator.setLength(leftMotor.getInputs().position + 0.1); // Add 0.1 for visual base

        if (leftMotor.getInputs().controlMode.startsWith("MM_")) {
            // If the motor is using Motion Magic (PID to a target), show the target and P/I/D/FF bars
            goalElevator.setLineWeight(6);
            pAmount.setLineWeight(6);
            dAmount.setLineWeight(6);
            fAmount.setLineWeight(6);
            iAmount.setLineWeight(6);

            // Set the target height and how big each control term is (scaled down for drawing)
            goalElevator.setLength(leftMotor.getInputs().setpoint + 0.1); // Add 0.1 for visual base
            pAmount.setLength(leftMotor.getInputs().propOutput / 100);
            dAmount.setLength(leftMotor.getInputs().derivOutput / 100);
            fAmount.setLength(leftMotor.getInputs().feedforward / 100);
            iAmount.setLength(leftMotor.getInputs().intOutput / 100);
        } else {
            // Hide the target and P/I/D/FF bars when not using Motion Magic
            goalElevator.setLineWeight(0);
            pAmount.setLineWeight(0);
            dAmount.setLineWeight(0);
            fAmount.setLineWeight(0);
            iAmount.setLineWeight(0);
        }

        // 3) Send the mechanism drawing to the logs/dashboard
        Logger.recordOutput("Elevator/Visualization", mech);

        // 4) Read tuning numbers and apply them to the motor controllers (units noted above)
        leftMotor.setkP(Constants.kP.get());
        leftMotor.setkD(Constants.kD.get());
        leftMotor.setkG(Constants.kG.get());
        leftMotor.setkS(Constants.kS.get());
        leftMotor.setkV(Constants.kV.get());
        leftMotor.setkA(Constants.kA.get());
        leftMotor.setkI(Constants.kI.get());
        leftMotor.setMaxVelocity(Constants.maxVelocity.get());
        leftMotor.setMaxAccel(Constants.maxAccel.get());
    }
}
