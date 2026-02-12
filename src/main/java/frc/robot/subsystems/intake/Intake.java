package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import frc.robot.io.BitIO;
import frc.robot.io.MotorIO;

public class Intake extends SubsystemBase {
    public static class Constants {
        public static final int intakeMotorId = 27;
        public static final int hingeMotorId = 28;
        // These are DIO IDs, separate from CAN IDs
        public static final int rightSwitchId = 1;
        public static final int leftSwitchId = 2;

        public static final double defaultSpeed = 0.1;

        public static final LoggedNetworkBoolean intakeLocked =
            new LoggedNetworkBoolean("Intake/Locked", true); // Toggle to enable braking of the hinge when stopped

        public static final LoggedNetworkBoolean intakeDisabled = new LoggedNetworkBoolean(
                "Intake/Disabled", false); // Toggle to completely disable all motors in the intake subsystem
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

    private void setDisabled(boolean disabled){
        hingeMotor.setDisabled(disabled);
        intakeMotor.setDisabled(disabled);
    }

    public void setHingeSpeed(double speed) {
        hingeMotor.setDutyCycle(speed);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.setDutyCycle(speed);
    }

    public void stop() {
        hingeMotor.setDutyCycle(0);
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
    }
}
