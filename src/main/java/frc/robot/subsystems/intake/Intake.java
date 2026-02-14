package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

        public static final LoggedNetworkNumber hingeKP = new LoggedNetworkNumber("Intake/HingeKP", 10);
        public static final LoggedNetworkNumber hingeKI = new LoggedNetworkNumber("Intake/HingeKI", 0);
        public static final LoggedNetworkNumber hingeKD = new LoggedNetworkNumber("Intake/HingeKD", 5);
        public static final LoggedNetworkNumber hingeKG = new LoggedNetworkNumber("Intake/HingeKG", 5);
        public static final LoggedNetworkNumber hingeKS = new LoggedNetworkNumber("Intake/HingeKS", 0);
        public static final LoggedNetworkNumber hingeKV = new LoggedNetworkNumber("Intake/HingeKV", 0);
        public static final LoggedNetworkNumber hingeKA = new LoggedNetworkNumber("Intake/HingeKA", 0);

        public static final LoggedNetworkBoolean intakeLocked =
                new LoggedNetworkBoolean("Intake/Locked", true); // Toggle to enable braking of the hinge when stopped

        public static final LoggedNetworkBoolean intakeDisabled = new LoggedNetworkBoolean(
                "Intake/Disabled", false); // Toggle to completely disable all motors in the intake subsystem

        public static final double hingeUp = Units.degreesToRadians(90);
        public static final double hingeDown = 0;

        public static final double intakeRatio = 1;
        public static final double hingeRatio = 15;

        // Simulation only
        public static final double intakeInertia = 0.000132; // kg m^2
        public static final double hingeInertia = 0.3; // kg m^2
    }

    private MotorIO hingeMotor;
    private MotorIO intakeMotor;
    private BitIO rightLimitSwitch;
    private BitIO leftLimitSwitch;

    public Intake(MotorIO intakeMotorIO, MotorIO hingeMotorIO, BitIO rightLimitSwitchIO, BitIO leftLimitSwitchIO) {
        hingeMotor = hingeMotorIO;
        rightLimitSwitch = rightLimitSwitchIO;
        leftLimitSwitch = leftLimitSwitchIO;
        intakeMotor = intakeMotorIO;

        hingeMotor.connectForwardLimitSwitch(rightLimitSwitch);
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

    public void setHingeGoal(double goal) {
        hingeMotor.setGoalWithCurrent(goal);
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
        hingeMotor.setkG(Constants.hingeKG.get());
        hingeMotor.setkV(Constants.hingeKV.get());
        hingeMotor.setkA(Constants.hingeKA.get());
    }
}
