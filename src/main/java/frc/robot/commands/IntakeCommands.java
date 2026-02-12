package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {

    private Intake intake;

    public IntakeCommands(Intake intake) {
        this.intake = intake;
    }

    // Stops all swerve output
    public Command stop() {
        return new InstantCommand(() -> intake.stop());
    }

    public Command setPos(double speed) {
        return new InstantCommand(() -> intake.setPos(speed));
    }

    public Command setDown(double speed) {
        return new InstantCommand(() -> intake.setDown(speed));
    }

    public Command setUp(double speed) {
        return new InstantCommand(() -> intake.setForward(speed));
    }

    public Command setSpeed(double speed) {
        return new InstantCommand(() -> intake.setSpeed(speed));
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
