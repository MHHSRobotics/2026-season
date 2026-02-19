package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.hopper.Hopper;

public class HopperCommands {
    public Hopper hopper;

    public HopperCommands(Hopper hopper) {
        this.hopper = hopper;
    }

    public Command setSpeed(DoubleSupplier speed) {
        return Commands.runEnd(() -> hopper.setSpeed(speed.getAsDouble()), () -> hopper.stop(), hopper);
    }

    // Spins hopper motor toward the shooter
    public Command forward() {
        return Commands.startEnd(() -> hopper.forward(), () -> hopper.stop(), hopper);
    }

    // Spins hopper motor toward the intake to keep fuel in
    public Command reverse() {
        return Commands.startEnd(() -> hopper.reverse(), () -> hopper.stop(), hopper);
    }
}
