package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {
    private Shooter shooter;

    public ShooterCommands(Shooter shooter) {
        this.shooter = shooter;
    }

    public Command flyShoot() {
        return new InstantCommand(() -> shooter.flyShoot());
    }

    public Command flyReverse() {
        return new InstantCommand(() -> shooter.flyReverse());
    }

    public Command flyStop() {
        return new InstantCommand(() -> shooter.flyStop());
    }

    public Command feedShoot() {
        return new InstantCommand(() -> shooter.feedShoot());
    }

    public Command feedReverse() {
        return new InstantCommand(() -> shooter.feedReverse());
    }

    public Command feedStop() {
        return new InstantCommand(() -> shooter.feedStop());
    }
}
