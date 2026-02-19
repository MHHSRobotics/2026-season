package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class MultiCommands {
    private HopperCommands hopper;
    private IntakeCommands intake;
    private ShooterCommands shooter;

    public MultiCommands(HopperCommands hopper, IntakeCommands intake, ShooterCommands shooter) {
        this.hopper = hopper;
        this.intake = intake;
        this.shooter = shooter;
    }

    public Command shoot() {
        return hopper.forward().alongWith(shooter.feedShoot(), shooter.flyShoot());
    }
}
