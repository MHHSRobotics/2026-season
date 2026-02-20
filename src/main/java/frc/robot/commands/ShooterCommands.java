package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {
    private Shooter shooter;

    public ShooterCommands(Shooter shooter) {
        this.shooter = shooter;
    }

    public Command setFlySpeed(DoubleSupplier speed) {
        return Commands.runEnd(() -> shooter.setFlyTargetSpeed(speed.getAsDouble()), () -> shooter.flyStop());
    }

    public Command flyShoot() {
        return Commands.startEnd(() -> shooter.flyShoot(), () -> shooter.flyStop());
    }

    public Command setFeedSpeed(DoubleSupplier speed) {
        return Commands.runEnd(() -> shooter.setFeedSpeed(speed.getAsDouble()), () -> shooter.feedStop());
    }

    public Command feedShootWhenAtTarget() {
        return Commands.startEnd(
                () -> {
                    if (shooter.atTargetSpeed()) {
                        shooter.feedShoot();
                    } else {
                        shooter.feedStop();
                    }
                },
                () -> shooter.feedStop());
    }

    public Command feedShoot() {
        return Commands.startEnd(() -> shooter.feedShoot(), () -> shooter.feedStop());
    }

    public Command feedReverse() {
        return Commands.startEnd(() -> shooter.feedReverse(), () -> shooter.feedStop());
    }
}
