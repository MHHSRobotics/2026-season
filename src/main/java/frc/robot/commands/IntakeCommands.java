package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Constants;

public class IntakeCommands {

    private Intake intake;

    public IntakeCommands(Intake intake) {
        this.intake = intake;
    }

    public Command hingeDown() {
        return new InstantCommand(() -> intake.setHingeGoal(Constants.hingeDown));
    }

    public Command hingeUp() {
        return new InstantCommand(() -> intake.setHingeGoal(Constants.hingeUp));
    }

    public Command setHingeSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> intake.setHingeSpeed(speed.getAsDouble()));
    }

    public Command hingeStop() {
        return new InstantCommand(() -> intake.hingeStop());
    }

    public Command setIntakeSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> intake.setIntakeSpeed(speed.getAsDouble()));
    }

    public Command intake() {
        return new InstantCommand(() -> intake.intake());
    }

    public Command outtake() {
        return new InstantCommand(() -> intake.outtake());
    }

    public Command intakeStop() {
        return new InstantCommand(() -> intake.intakeStop());
    }
}
