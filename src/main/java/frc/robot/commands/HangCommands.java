package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.hang.Hang;

public class HangCommands {
    private Hang hang;

    public HangCommands(Hang hang) {
        this.hang = hang;
    }

    public Command setSpeed(DoubleSupplier speed) {
        return Commands.runEnd(() -> hang.setSpeed(speed.getAsDouble()), () -> hang.setSpeed(0), hang);
    }
}
