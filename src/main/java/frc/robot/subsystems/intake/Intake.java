package frc.robot.subsystems.intake;

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

import frc.robot.Constants.Mode;
import frc.robot.io.BitIO;
import frc.robot.io.MotorIO;

public class Intake extends SubsystemBase {
    public static class Constants {
        public static final int intakeMotorId = 22;
        public static final int hingeMotorId = 23;
        // These are DIO IDs, separate from CAN IDs
        public static final int rightSwitchId = 1;
        public static final int leftSwitchId = 2;

        public static final double defaultSpeed = 0.5;

        public static final LoggedNetworkNumber hingeKP = new LoggedNetworkNumber("Intake/Hinge/kP", 10);
        public static final LoggedNetworkNumber hingeKI = new LoggedNetworkNumber("Intake/Hinge/kI", 0);
        public static final LoggedNetworkNumber hingeKD = new LoggedNetworkNumber("Intake/Hinge/kD", 5);
        public static final LoggedNetworkNumber hingeKG =
                new LoggedNetworkNumber("Intake/Hinge/kG", frc.robot.Constants.currentMode == Mode.SIM ? 0 : 5);
        public static final LoggedNetworkNumber hingeKS = new LoggedNetworkNumber("Intake/Hinge/kS", 0);
        public static final LoggedNetworkNumber hingeKV = new LoggedNetworkNumber("Intake/Hinge/kV", 0);
        public static final LoggedNetworkNumber hingeKA = new LoggedNetworkNumber("Intake/Hinge/kA", 0);

        public static final LoggedNetworkNumber hingeMaxVel = new LoggedNetworkNumber("Intake/Hinge/maxVel", 5);
        public static final LoggedNetworkNumber hingeMaxAccel = new LoggedNetworkNumber("Intake/Hinge/maxAccel", 10);

        public static final LoggedNetworkNumber hingeVerticalPos =
                new LoggedNetworkNumber("Intake/Hinge/VerticalPos", 0);

        public static final LoggedNetworkBoolean intakeLocked =
                new LoggedNetworkBoolean("Intake/Locked", true); // Toggle to enable braking of the hinge when stopped

        public static final LoggedNetworkBoolean intakeDisabled = new LoggedNetworkBoolean(
                "Intake/Disabled", false); // Toggle to completely disable all motors in the intake subsystem

        public static final double hingeUp = Units.degreesToRadians(90);
        public static final double hingeDown = 0;

        public static final double intakeRatio = 1;
        public static final double hingeRatio = 15;

        public static final boolean hingeInverted = false;
        public static final boolean intakeInverted = false;

        // Simulation only
        public static final double intakeInertia = 0.000132; // kg m^2
        public static final double hingeInertia = 0.3; // kg m^2

    }

    private MotorIO hingeMotor;
    private MotorIO intakeMotor;
    private BitIO rightLimitSwitch;
    private BitIO leftLimitSwitch;

    private boolean intakeUp = true;

    public Intake(MotorIO intakeMotorIO, MotorIO hingeMotorIO, BitIO rightLimitSwitchIO, BitIO leftLimitSwitchIO) {
        hingeMotor = hingeMotorIO;
        rightLimitSwitch = rightLimitSwitchIO;
        leftLimitSwitch = leftLimitSwitchIO;
        intakeMotor = intakeMotorIO;

        hingeMotor.setInverted(Constants.hingeInverted);
        hingeMotor.connectInternalSensor(Constants.hingeRatio);
        hingeMotor.setPosition(Constants.hingeUp);
        // hingeMotor.connectForwardLimitSwitch(rightLimitSwitch);

        intakeMotor.setInverted(Constants.intakeInverted);
        intakeMotor.connectInternalSensor(Constants.intakeRatio);
    }

    private void setLocked(boolean brake) {
        hingeMotor.setBraking(brake);
    }

    private void setDisabled(boolean disabled) {
        hingeMotor.setDisabled(disabled);
        intakeMotor.setDisabled(disabled);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.setDutyCycle(speed);
    }

    public void hingeStop() {
        hingeMotor.setDutyCycle(0);
    }

    public void setHingeSpeed(double speed) {
        hingeMotor.setDutyCycle(speed);
    }

    public void switchPos() {
        if (intakeUp == false) {
            setHingeGoal(Constants.hingeUp);
        } else {
            setHingeGoal(Constants.hingeDown);
        }
    }

    public boolean getIntakePos() {
        return intakeUp;
    }

    public void setHingeGoal(double goal) {
        intakeUp = false;
        if (goal > 0) {
            intakeUp = true;
        }
        hingeMotor.setGoalWithCurrentMagic(
                goal,
                () -> Constants.hingeKG.get()
                        * Math.cos(hingeMotor.getInputs().position - Constants.hingeVerticalPos.get()));
    }

    public void intake() {
        intakeMotor.setDutyCycle(Constants.defaultSpeed);
    }

    public void outtake() {
        intakeMotor.setDutyCycle(-Constants.defaultSpeed);
    }

    public void intakeStop() {
        intakeMotor.setDutyCycle(0);
    }

    @Override
    public void periodic() {
        setLocked(Constants.intakeLocked.get());
        setDisabled(Constants.intakeDisabled.get());

        intakeMotor.update();
        hingeMotor.update();
        rightLimitSwitch.update();
        leftLimitSwitch.update();

        hingeMotor.setkP(Constants.hingeKP.get());
        hingeMotor.setkI(Constants.hingeKI.get());
        hingeMotor.setkD(Constants.hingeKD.get());
        hingeMotor.setkS(Constants.hingeKS.get());
        hingeMotor.setkV(Constants.hingeKV.get());
        hingeMotor.setkA(Constants.hingeKA.get());
        hingeMotor.setMaxVelocity(Constants.hingeMaxVel.get());
        hingeMotor.setMaxAccel(Constants.hingeMaxAccel.get());

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
