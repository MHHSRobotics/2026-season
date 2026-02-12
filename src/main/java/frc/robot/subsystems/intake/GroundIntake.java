package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import frc.robot.io.BitIO;
import frc.robot.io.MotorIO;
import frc.robot.subsystems.intake.GroundIntake.Constants.MechanicalSwitchState;

public class GroundIntake extends SubsystemBase {
    public static class Constants {
        public static enum MechanicalSwitchState {
            HIGH_POS,
            NEUTRAL
        }
        public static final int id = 15;
        public static final int motorId = 16;
        public static final double defaultSpeed = 0.1;
    }

    private MechanicalSwitchState currentState = MechanicalSwitchState.HIGH_POS;
    private MotorIO hingeMotor;
    private final MotorIO intakeMotor;
    private BitIO rightLimitSwitch;
    private BitIO leftLimitSwitch;

    public static final LoggedNetworkBoolean gIntakeLocked =
            new LoggedNetworkBoolean("GIntake/Locked", true); // Toggle to enable braking when stopped

    public static final LoggedNetworkBoolean gIntakeDisabled = new LoggedNetworkBoolean(
            "GIntake/Disabled", false); // Toggle to completely disable all motors in the swerve subsystem

    public GroundIntake(MotorIO intakeMotorIO, MotorIO hingeMotorIO, BitIO rightLimitSwitchIO, BitIO leftLimitSwitchIO) {
        hingeMotor = hingeMotorIO;
        rightLimitSwitch = rightLimitSwitchIO;
        leftLimitSwitch = leftLimitSwitchIO;
        intakeMotor = intakeMotorIO;
    }

    public void setLocked(boolean brake) {
        stop();
        hingeMotor.setBraking(brake);
    }

    public void setSpeed(double speed) {
        hingeMotor.setDutyCycle(speed);
    }

    public void setForward(double speed) {
        if (leftLimitSwitch.getInputs().value || rightLimitSwitch.getInputs().value) {
            stop();
            setLocked(true);
            currentState = MechanicalSwitchState.HIGH_POS;
        } else {
            hingeMotor.setDutyCycle(speed);
        }
    }

    public void setDown(double speed) {
        currentState = MechanicalSwitchState.NEUTRAL;
        setLocked(false);
        hingeMotor.setDutyCycle(speed);

        try {
            Thread.sleep(1000); // One second delay
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        stop();
    }

    public void setPos(double speed) {
        if (currentState == MechanicalSwitchState.NEUTRAL) {
            setForward(speed);
        } else {
            setDown(speed);
        }
    }

    public void stop() {
        hingeMotor.setDutyCycle(0);
    }

    /** Turn intake motor ON (forward only) */
    public void intakeOn() {
        intakeMotor.setDutyCycle(Constants.defaultSpeed); // adjust speed as needed
    }

    public void intakeReverse() {
        intakeMotor.setDutyCycle(-Constants.defaultSpeed); // adjust speed as needed
    }

    /** Turn intake motor OFF */
    public void intakeOff() {
        intakeMotor.setDutyCycle(0);
    }

    @Override
    public void periodic() {
        hingeMotor.setBraking(gIntakeLocked.get());

        hingeMotor.setDisabled(gIntakeDisabled.get());

        hingeMotor.update();

        rightLimitSwitch.update();

        leftLimitSwitch.update();

        intakeMotor.setBraking(gIntakeLocked.get());
        intakeMotor.setDisabled(gIntakeDisabled.get());
        intakeMotor.update();
    }
}
