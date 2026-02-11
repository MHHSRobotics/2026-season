package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import frc.robot.io.MotorIO;

public class IntakeMotor extends SubsystemBase {
    public static class Constants {
        public static LoggedNetworkBoolean locked = new LoggedNetworkBoolean("Intake/Locked", false);
        public static LoggedNetworkBoolean disabled = new LoggedNetworkBoolean("Intake/Disabled", false);
        public static final double defaultSpeed = 0.1;
    }

    private final MotorIO intakeMotor;

    public IntakeMotor(MotorIO intakeMotorIO) {

        intakeMotor = intakeMotorIO;
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
        intakeMotor.setBraking(Constants.locked.get());
        intakeMotor.setDisabled(Constants.disabled.get());
        intakeMotor.update();
    }
}
