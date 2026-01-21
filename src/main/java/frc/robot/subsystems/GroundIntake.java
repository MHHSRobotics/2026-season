package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

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
    private int id = 0;
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    public void setLocked(boolean brake) {
        hingeMotor.setBraking(brake);
    }

    public void setDisabled(boolean brake) {
        hingeMotor.setBraking(brake);
    }

    public GroundIntake(
            MotorIO hingeMotorIO,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;
        hingeMotor = hingeMotorIO;
    }

    public void setForward(double radPerSecond) {
        hingeMotor.setVelocityWithVoltage(radPerSecond);
        currentState = MechanicalSwitchState.NEUTRAL;
        if (connectForwardLimitSwitch(id) == false) {
            setLocked(false);
        }
    }

    public void stop() {
        hingeMotor.setVoltage(0);
    }

    public boolean connectForwardLimitSwitch(int id) {
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = id;
        return (config.HardwareLimitSwitch.ForwardLimitEnable);
    }

    public void setPointUp(double radiansPerSecond) {
        setForward(radiansPerSecond);
        connectForwardLimitSwitch(id);
        if (connectForwardLimitSwitch(id) == true) {
            currentState = MechanicalSwitchState.HIGH_POS;
            setLocked(true);
        }
    }
}
