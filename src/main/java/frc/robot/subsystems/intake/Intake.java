package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants.Mode;
import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class Intake extends SubsystemBase {
    public static class Constants {
        public static final int rollerMotorId = 14;
        public static final int hingeMotorId = 15;
        public static final int hingeEncoderId = 19;
        // These are DIO IDs, separate from CAN IDs
        public static final int rightSwitchId = 1;
        public static final int leftSwitchId = 2;

        public static final LoggedNetworkNumber defaultSpeed =
                new LoggedNetworkNumber("Intake/RollerSpeed", frc.robot.Constants.currentMode == Mode.SIM ? 288 : 312);

        public static final LoggedNetworkNumber outtakeSpeed = new LoggedNetworkNumber("Intake/OuttakeSpeed", 300);

        public static final LoggedNetworkNumber minSpeed = new LoggedNetworkNumber("Intake/MinSpeed", 240);

        public static final LoggedNetworkNumber flyKP = new LoggedNetworkNumber("Intake/Roller/kP", 1.5);
        public static final LoggedNetworkNumber flyKD = new LoggedNetworkNumber("Intake/Roller/kD", 0.03);

        public static final LoggedNetworkNumber hingeKP =
                new LoggedNetworkNumber("Intake/Hinge/kP", frc.robot.Constants.currentMode == Mode.SIM ? 30 : 35);

        public static final LoggedNetworkNumber hingeKI = new LoggedNetworkNumber("Intake/Hinge/kI", 0);
        public static final LoggedNetworkNumber hingeKD =
                new LoggedNetworkNumber("Intake/Hinge/kD", frc.robot.Constants.currentMode == Mode.SIM ? 25 : 13);
        public static final LoggedNetworkNumber hingeKG =
                new LoggedNetworkNumber("Intake/Hinge/kG", frc.robot.Constants.currentMode == Mode.SIM ? 25 : 21.75);
        public static final LoggedNetworkNumber hingeKS = new LoggedNetworkNumber("Intake/Hinge/kS", 5.75);
        public static final LoggedNetworkNumber hingeKV = new LoggedNetworkNumber("Intake/Hinge/kV", 0);
        public static final LoggedNetworkNumber hingeKA = new LoggedNetworkNumber("Intake/Hinge/kA", 0);

        public static final LoggedNetworkNumber hingeMaxVel = new LoggedNetworkNumber("Intake/Hinge/maxVel", 5);
        public static final LoggedNetworkNumber hingeMaxAccel = new LoggedNetworkNumber("Intake/Hinge/maxAccel", 10);

        public static final LoggedNetworkNumber hingeVerticalPos = new LoggedNetworkNumber(
                "Intake/Hinge/VerticalPos", frc.robot.Constants.currentMode == Mode.SIM ? 1.34 : 1.28);

        public static final LoggedNetworkBoolean intakeLocked =
                new LoggedNetworkBoolean("Intake/Locked", true); // Toggle to enable braking of the hinge when stopped

        public static final LoggedNetworkBoolean intakeDisabled = new LoggedNetworkBoolean(
                "Intake/Disabled", false); // Toggle to completely disable all motors in the intake subsystem

        public static final double rollerCurrentLimit = 150;

        public static final double hingeDown = Units.degreesToRadians(0);
        public static final double hingeUp =
                frc.robot.Constants.currentMode == Mode.SIM ? Units.degreesToRadians(90) : Units.degreesToRadians(91.7);

        public static final double rollerRatio = 1.25;
        public static final double hingeRatio = 15;
        public static final double encoderRatio = 1;

        public static final boolean hingeInverted = true;
        public static final boolean rollerInverted = false;
        public static final boolean encoderInverted = false;

        public static final double hingeOffset = -0.5;

        public static final LoggedNetworkNumber hingeDownTorque = new LoggedNetworkNumber("Intake/HingeDown", 20);

        public static final double rollerRadius = 0.0286; // 1.125 inches
        // Simulation only
        public static final double rollerInertia = 0.000132; // kg m^2
        public static final double hingeInertia = 0.3; // kg m^2
    }

    private MotorIO hingeMotor;
    private MotorIO rollerMotor;
    private EncoderIO hingeEncoder;

    private boolean intakeUp = true;

    // Hinge only travels ~90 degrees and is heavy (0.3 kg*m^2), so use a gentler ramp/step and a
    // shorter timeout than the 1V/s, 7V, 10s defaults - the soft limits below are the real
    // backstop, but there's no reason to build up more voltage/speed than needed for a good fit.
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(3).per(Second), Volts.of(14), Seconds.of(20), null),
            new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motor(s).
                    (voltage) -> hingeMotor.setTorqueCurrent(voltage.in(Volts)),
                    // AdvantageKit already logs appliedVoltage/position/velocity every cycle via the IO
                    // layer, so this can stay null; use AdvantageScope's SysId tool on those logged fields.
                    log -> {
                        // Record a frame for the shooter motor.
                        log.motor("intake-hinge");
                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test state in
                    // WPILog with this subsystem's name ("shooter")
                    this));

    public Intake(MotorIO rollerMotorIO, MotorIO hingeMotorIO, EncoderIO hingeEncoderIO) {
        hingeMotor = hingeMotorIO;
        rollerMotor = rollerMotorIO;
        hingeEncoder = hingeEncoderIO;

        hingeEncoder.setInverted(Constants.encoderInverted);
        hingeEncoder.setGearRatio(Constants.encoderRatio);

        hingeMotor.setInverted(Constants.hingeInverted);
        hingeMotor.connectEncoder(hingeEncoder, Constants.hingeRatio, true);
        hingeMotor.setFeedforwardType(GravityTypeValue.Arm_Cosine);
        hingeMotor.setStaticFeedforwardType(StaticFeedforwardSignValue.UseClosedLoopSign);
        hingeMotor.setOffset(Constants.hingeOffset);
        hingeMotor.setLimits(Constants.hingeDown, Constants.hingeUp);

        rollerMotor.setInverted(Constants.rollerInverted);
        rollerMotor.connectInternalSensor(Constants.rollerRatio);
        rollerMotor.setStatorCurrentLimit(Constants.rollerCurrentLimit);
    }

    private void setLocked(boolean brake) {
        hingeMotor.setBraking(brake);
    }

    private void setDisabled(boolean disabled) {
        hingeMotor.setDisabled(disabled);
        rollerMotor.setDisabled(disabled);
    }

    public void setRollerSpeed(double speed) {
        rollerMotor.setDutyCycle(speed);
    }

    public void setRollerTargetSpeed(double speed) {
        rollerMotor.setVelocityWithCurrent(speed);
    }

    public void setHingeSpeed(double speed) {
        hingeMotor.setDutyCycle(speed);
    }

    public void switchPos() {
        if (intakeUp == false) {
            setHingeUp();
        } else {
            setHingeDown();
        }
    }

    public boolean isIntakeUp() {
        return intakeUp;
    }

    public void setHingeDown() {
        intakeUp = false;
        setHingeGoal(Constants.hingeDown);
    }

    public void setHingeUp() {
        intakeUp = true;
        setHingeGoal(Constants.hingeUp);
    }

    public void setHingeGoal(double goal) {
        hingeMotor.setGoalWithCurrentMagic(goal, () -> {
            double position = hingeMotor.getInputs().position;
            if (position > 0 || goal > 0) {
                double gravityFF =
                        Constants.hingeKG.get() * Math.cos(position + Math.PI / 2 - Constants.hingeVerticalPos.get());
                return gravityFF;
            } else {
                return -Constants.hingeDownTorque.get();
            }
        });
    }

    public double getHingeGoal() {
        return hingeMotor.getInputs().setpoint;
    }

    public void rollerStop() {
        rollerMotor.setDutyCycle(0);
    }

    // Margin kept clear of the soft limits so a test ends itself before ever reaching them.
    private static final double sysIdSafetyMargin = Units.degreesToRadians(10);

    // Stops a SysId test automatically once the hinge nears the limit it's moving toward, instead
    // of relying only on the operator releasing the button in time.
    private boolean nearSysIdLimit(SysIdRoutine.Direction direction) {
        double position = hingeMotor.getInputs().position;
        return direction == SysIdRoutine.Direction.kForward
                ? position > Constants.hingeUp - sysIdSafetyMargin
                : position < Constants.hingeDown + sysIdSafetyMargin;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine
                .quasistatic(direction)
                .beforeStarting(Commands.runOnce(() -> Logger.recordOutput(
                        "Intake/sysidstate",
                        direction == SysIdRoutine.Direction.kForward ? "quasistatic-forward" : "quasistatic-reverse")))
                .until(() -> nearSysIdLimit(direction))
                .andThen(Commands.runOnce(() -> Logger.recordOutput("Intake/sysidstate", "")));
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine
                .dynamic(direction)
                .beforeStarting(Commands.runOnce(() -> Logger.recordOutput(
                        "Intake/sysidstate",
                        direction == SysIdRoutine.Direction.kForward ? "dynamic-forward" : "dynamic-reverse")))
                .until(() -> nearSysIdLimit(direction))
                .andThen(Commands.runOnce(() -> Logger.recordOutput("Intake/sysidstate", "")));
    }

    @Override
    public void periodic() {
        setLocked(Constants.intakeLocked.get());
        setDisabled(Constants.intakeDisabled.get());

        rollerMotor.update();
        hingeMotor.update();
        hingeEncoder.update();

        hingeMotor.setkP(Constants.hingeKP.get());
        hingeMotor.setkI(Constants.hingeKI.get());
        hingeMotor.setkD(Constants.hingeKD.get());
        hingeMotor.setkS(Constants.hingeKS.get());
        hingeMotor.setkV(Constants.hingeKV.get());
        hingeMotor.setkA(Constants.hingeKA.get());
        hingeMotor.setMaxVelocity(Constants.hingeMaxVel.get());
        hingeMotor.setMaxAccel(Constants.hingeMaxAccel.get());

        rollerMotor.setkP(Constants.flyKP.get());
        rollerMotor.setkD(Constants.flyKD.get());

        updateVis();

        Logger.recordOutput("IntakeUp", intakeUp);
    }

    // On-screen drawing of the wrist for dashboards (length is visual only)
    private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);

    // The fixed base point for the wrist drawing
    private final LoggedMechanismRoot2d root = mech.getRoot("HingeRoot", 1, 1.5);

    // The live wrist drawing that rotates to match the wrist angle (radians)
    private final LoggedMechanismLigament2d hinge =
            root.append(new LoggedMechanismLigament2d("Hinge", 0.5, 0, 6, new Color8Bit(Color.kOrange)));

    // Drawing that shows the wrist's target angle (radians)
    private final LoggedMechanismLigament2d goalHinge =
            root.append(new LoggedMechanismLigament2d("GoalHinge", 0.5, 0, 6, new Color8Bit(Color.kYellow)));

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

    private void updateVis() {
        // 2) Update the on-screen wrist drawing to match the current wrist angle (radians)
        hinge.setAngle(Rotation2d.fromRadians(hingeMotor.getInputs().position));

        if (hingeMotor.getInputs().controlMode.startsWith("MM_")) {
            // If the motor is using Motion Magic (PID to a target), show the target and P/I/D/FF bars
            goalHinge.setLineWeight(6);
            pAmount.setLineWeight(6);
            dAmount.setLineWeight(6);
            fAmount.setLineWeight(6);
            iAmount.setLineWeight(6);

            // Set the target angle and how big each control term is (scaled down for drawing)
            goalHinge.setAngle(Rotation2d.fromRadians(hingeMotor.getInputs().setpoint));
            pAmount.setLength(hingeMotor.getInputs().propOutput / 100);
            dAmount.setLength(hingeMotor.getInputs().derivOutput / 100);
            fAmount.setLength(hingeMotor.getInputs().feedforward / 100);
            iAmount.setLength(hingeMotor.getInputs().intOutput / 100);
        } else {
            // Hide the target and P/I/D/FF bars when not using Motion Magic
            goalHinge.setLineWeight(0);
            pAmount.setLineWeight(0);
            dAmount.setLineWeight(0);
            fAmount.setLineWeight(0);
            iAmount.setLineWeight(0);
        }

        // 3) Send the mechanism drawing to the logs/dashboard
        Logger.recordOutput("Intake/Visualization", mech);
    }
}
