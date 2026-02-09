package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeMotor extends SubsystemBase {
    public static final int INTAKE_MOTOR_CAN_ID = 1;

    private final TalonFX intakeMotor;
    private final DutyCycleOut duty = new DutyCycleOut(0);

    public IntakeMotor() {
        intakeMotor = new TalonFX(INTAKE_MOTOR_CAN_ID);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        // intakeMotor.setInverted(false);
    }

    /** Turn intake motor ON (forward only) */
    public void intakeOn() {
        intakeMotor.setControl(duty.withOutput(0.8)); // adjust speed as needed
    }

    /** Turn intake motor OFF */
    public void intakeOff() {
        intakeMotor.setControl(duty.withOutput(0.0));
    }
}
