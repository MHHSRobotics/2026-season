package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Constants;

public class IntakeCommands {

    private Intake intake;

    public IntakeCommands(Intake intake) {
        this.intake = intake;
    }

    public Command hingeDown() {
        return new InstantCommand(() -> intake.setHingeGoal(Constants.hingeDown)).withName("hinge down");
    }

    public Command hingeUp() {
        return new InstantCommand(() -> intake.setHingeGoal(Constants.hingeUp)).withName("hinge up");
    }

    public Command switchHinge() {
        return new InstantCommand(() -> intake.switchPos()).withName("switch hinge");
    }

    public Command changeGoal(DoubleSupplier change) {
        return Commands.run(() -> intake.setHingeGoal(intake.getHingeGoal() + change.getAsDouble()), intake)
                .withName("change goal");
    }

    public Command setHingeSpeed(DoubleSupplier speed) {
        return Commands.runEnd(() -> intake.setHingeSpeed(speed.getAsDouble()), () -> intake.hingeStop(), intake)
                .withName("set hinge speed");
    }

    public Command setIntakeSpeed(DoubleSupplier speed) {
        return Commands.runEnd(() -> intake.setIntakeSpeed(speed.getAsDouble()), () -> intake.intakeStop(), intake)
                .withName("set intake speed");
    }

    public Command intake() {
        return Commands.startEnd(() -> intake.intake(), () -> intake.intakeStop(), intake)
                .withName("intake");
    }

    public Command outtake() {
        return Commands.startEnd(() -> intake.outtake(), () -> intake.intakeStop(), intake)
                .withName("outtake");
    }
}
