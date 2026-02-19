package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.hang.Hang;

public class HangCommands {
    private Hang hang;

    public HangCommands(Hang hang) {
        this.hang = hang;
    }

    public Command setSpeed(DoubleSupplier speed) {
        return Commands.runEnd(() -> hang.setSpeed(speed.getAsDouble()),()->hang.stop(), hang);
    }

    public Command moveUp() {
        return Commands.startEnd(() -> hang.moveUp(),()->hang.stop(), hang);
    }

    public Command moveDown() {
        return Commands.startEnd(()->hang.moveDown(),() -> hang.moveDown(), hang);
    }
}
