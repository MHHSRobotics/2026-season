package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.hang.Hang;

public class HangCommands {
    private Hang hang;

    public HangCommands(Hang hang) {
        this.hang = hang;
    }

    public Command setSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> hang.setSpeed(speed.getAsDouble()), hang);
    }

    public Command moveUp() {
        return new InstantCommand(() -> hang.moveUp(), hang);
    }

    public Command moveDown() {
        return new InstantCommand(() -> hang.moveDown(), hang);
    }

    public Command stop() {
        return new InstantCommand(() -> hang.stop(), hang);
    }
}
