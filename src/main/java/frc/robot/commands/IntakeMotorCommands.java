package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.intake.IntakeMotor;

public class IntakeMotorCommands {
    private IntakeMotor intake;

    public IntakeMotorCommands(IntakeMotor intake) {
        this.intake = intake;
    }

    public Command intakeForward() {
        return new InstantCommand(() -> intake.intakeOn());
    }

    public Command intakeReverse() {
        return new InstantCommand(() -> intake.intakeReverse());
    }

    public Command intakeStop() {
        return new InstantCommand(() -> intake.intakeOff());
    }
}
