package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.hang.Hang;


public class HangCommands {
    private Hang hang;

    public HangCommands(Hang hang) {
        this.hang = hang;
    }

    public Command moveUp() {
        return new InstantCommand(() -> hang.setSpeed(0.5), hang);
    }

    public Command moveDown() {
        return new InstantCommand(() -> hang.setSpeed(-0.5), hang);
    }

    public Command stop() {
        return new InstantCommand(() -> hang.setSpeed(0), hang);
    }
}
