package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.hopper.Hopper;

public class HopperCommands {
    public Hopper hopper;

    public HopperCommands(Hopper hopper) {
        this.hopper = hopper;
    }

    public Command rollForward() {
        return new InstantCommand(() -> hopper.setRollerForward(), hopper);
    }

    public Command rollReverse() {
        return new InstantCommand(() -> hopper.setRollerReverse(), hopper);
    }

    public Command stop() {
        return new InstantCommand(() -> hopper.setRollerStop(), hopper);
    }
}
