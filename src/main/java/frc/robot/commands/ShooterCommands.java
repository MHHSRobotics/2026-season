package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {
    private Shooter shooter;

    public ShooterCommands(Shooter shooter) {
        this.shooter = shooter;
    }

    public Command setFlySpeed(DoubleSupplier speed) {
        return Commands.runEnd(() -> shooter.setFlySpeed(speed.getAsDouble()),()->shooter.flyStop(), shooter);
    }

    public Command flyShoot() {
        return Commands.startEnd(() -> shooter.flyShoot(),()->shooter.flyStop(), shooter);
    }

    public Command flyReverse() {
        return Commands.startEnd(() -> shooter.flyReverse(),()->shooter.flyStop(), shooter);
    }

    public Command setFeedSpeed(DoubleSupplier speed) {
        return Commands.runEnd(() -> shooter.setFeedSpeed(speed.getAsDouble()),()->shooter.feedStop(), shooter);
    }

    public Command feedShoot() {
        return Commands.startEnd(() -> shooter.feedShoot(),()->shooter.feedStop(), shooter);
    }

    public Command feedReverse() {
        return Commands.startEnd(() -> shooter.feedReverse(),()->shooter.feedStop(), shooter);
    }
}
