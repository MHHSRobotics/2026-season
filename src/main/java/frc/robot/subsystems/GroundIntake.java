package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.io.MotorIO;

public class GroundIntake extends SubsystemBase {
    public static class Constants {
        public static enum MechanicalSwitchState {
            HIGH_POS,
            LOW_POS
        }
    }

    private MotorIO hingeMotor;
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;
    // private MechanicalSwitch mechanicalSwitch;

    public GroundIntake(
            MotorIO hingeMotorIO,
            // MechanicalSwitch mechanicalSwitchIO,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;
        hingeMotor = hingeMotorIO;

        // mechanicalSwitch = mechanicalSwitchIO

        hingeMotor.setBraking(true);
        /*/*
        hingeMotor.setGains(constants.hingeMotorGains);
        hingeMotor.setGearRatio(constants.hingeMotorGearRatio);
        hingeMotor.setStatorCurrentLimit(constants.SlipCurrent);
        hingeMotor.setInverted(constants.hingeMotorInverted);
        //* */

    }

    public void setForward(double radPerSecond) {
        hingeMotor.setVelocityWithVoltage(radPerSecond);
    }

    public void stop() {
        hingeMotor.setVoltage(0);
    }
}
