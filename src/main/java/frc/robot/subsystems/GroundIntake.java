package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.io.BitIODigitalSignal;
import frc.robot.io.MotorIO;
import frc.robot.subsystems.GroundIntake.Constants.MechanicalSwitchState;

public class GroundIntake extends SubsystemBase {
    public static class Constants {
        public static enum MechanicalSwitchState {
            HIGH_POS,
            NEUTRAL
        }
    }

    private MechanicalSwitchState currentState = MechanicalSwitchState.HIGH_POS;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private MotorIO hingeMotor;
    private BitIODigitalSignal rightLimitSwitch = new BitIODigitalSignal("MechanicalSwitchRight", "", 0);
    private BitIODigitalSignal leftLimitSwitch = new BitIODigitalSignal("MechanicalSwitchLeft", "", 1);
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    public GroundIntake(
            MotorIO hingeMotorIO,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;
        hingeMotor = hingeMotorIO;
    }

    public void setLocked(boolean brake) {
        stop();
        hingeMotor.setBraking(brake);
    }
    /*
    public void setForward(double radPerSecond) {
        if (leftLimitSwitch.getInputs() || rightLimitSwitch.getInputs()) {
            stop();
            setLocked(true);
            currentState = MechanicalSwitchState.HIGH_POS;
        } else {
            hingeMotor.setDutyCycle(radPerSecond);
        }
    }
    */

    public void setDown(double radPerSecond) {
        currentState = MechanicalSwitchState.NEUTRAL;
        setLocked(false);
        hingeMotor.setDutyCycle(radPerSecond);

        try {
            Thread.sleep(1000); // One second delay
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        stop();
    }

    public void stop() {
        hingeMotor.setDutyCycle(0);
    }

    @Override
    public void periodic() {}
}
