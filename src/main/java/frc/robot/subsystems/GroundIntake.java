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
    private int id1 = 0;
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

    public void setForward(double radPerSecond) {
        while (connectForwardLimitSwitch(id) == false || connectForwardLimitSwitch1(id1) == false) {
            hingeMotor.setDutyCycle(radPerSecond);
        }
        stop();
        setLocked(true);
        currentState = MechanicalSwitchState.HIGH_POS;
    }

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

    public boolean connectForwardLimitSwitch(int i) {
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = i;
        return (config.HardwareLimitSwitch.ForwardLimitEnable);
    }

    public boolean connectForwardLimitSwitch1(int i1) {
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = i1;
        return (config.HardwareLimitSwitch.ForwardLimitEnable);
    }

    @Override
    public void periodic() {}
}
