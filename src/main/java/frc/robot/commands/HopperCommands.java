package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.hopper.Hopper;

public class HopperCommands {
    public Hopper hopper;

    public HopperCommands(Hopper hopper) {
        this.hopper = hopper;
    }

    public Command setSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> hopper.setSpeed(speed.getAsDouble()), hopper);
    }

    // Spins hopper motor toward the shooter
    public Command forward() {
        return new InstantCommand(() -> hopper.forward(), hopper);
    }

    // Spins hopper motor toward the intake to keep fuel in
    public Command reverse() {
        return new InstantCommand(() -> hopper.reverse(), hopper);
    }

    // Stops hopper motor from spinning
    public Command stop() {
        return new InstantCommand(() -> hopper.stop(), hopper);
    }
}
