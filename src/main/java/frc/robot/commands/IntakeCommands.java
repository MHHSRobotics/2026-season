package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {

    private Intake intake;

    public IntakeCommands(Intake intake) {
        this.intake = intake;
    }

    public Command setHingeUpShort() {
        return Commands.startEnd(() -> intake.setHingeUp(), () -> intake.setHingeDown(), intake);
    }

    public Command setHingeDown() {
        return Commands.runOnce(() -> intake.setHingeDown());
    }

    public Command setHingeUp() {
        return Commands.runOnce(() -> intake.setHingeUp());
    }

    public Command switchHinge() {
        return new InstantCommand(() -> intake.switchPos()).withName("switch hinge");
    }

    public Command changeGoal(DoubleSupplier change) {
        return Commands.run(() -> intake.setHingeGoal(intake.getHingeGoal() + change.getAsDouble()), intake)
                .withName("change goal");
    }

    public Command setHingeSpeed(DoubleSupplier speed) {
        return Commands.runEnd(() -> intake.setHingeSpeed(speed.getAsDouble()), () -> intake.setHingeSpeed(0), intake)
                .withName("set hinge speed");
    }

    public Command setRollerSpeed(DoubleSupplier speed) {
        return Commands.runEnd(() -> intake.setRollerSpeed(speed.getAsDouble()), () -> intake.rollerStop(), intake)
                .withName("set intake speed");
    }

    public Command setRollerTargetSpeed(DoubleSupplier speed) {
        return Commands.runEnd(
                        () -> intake.setRollerTargetSpeed(speed.getAsDouble()), () -> intake.rollerStop(), intake)
                .withName("set intake target speed");
    }

    public Command intake() {
        return Commands.startEnd(
                        () -> intake.setRollerTargetSpeed(Intake.Constants.defaultSpeed.get()),
                        () -> intake.rollerStop(),
                        intake)
                .withName("intake");
    }

    public Command outtake() {
        return Commands.startEnd(
                        () -> intake.setRollerTargetSpeed(-Intake.Constants.outtakeSpeed.get()),
                        () -> intake.rollerStop(),
                        intake)
                .withName("outtake");
    }

    public Command rollerStop() {
        return Commands.runOnce(() -> intake.rollerStop(), intake);
    }
}
