package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {
    private Shooter shooter;

    public ShooterCommands(Shooter shooter) {
        this.shooter = shooter;
    }

    public Command setFlySpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> shooter.setFlySpeed(speed.getAsDouble()), shooter);
    }

    public Command flyShoot() {
        return new InstantCommand(() -> shooter.flyShoot(), shooter);
    }

    public Command flyReverse() {
        return new InstantCommand(() -> shooter.flyReverse(), shooter);
    }

    public Command flyStop() {
        return new InstantCommand(() -> shooter.flyStop(), shooter);
    }

    public Command setFeedSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> shooter.setFeedSpeed(speed.getAsDouble()), shooter);
    }

    public Command feedShoot() {
        return new InstantCommand(() -> shooter.feedShoot(), shooter);
    }

    public Command feedReverse() {
        return new InstantCommand(() -> shooter.feedReverse(), shooter);
    }

    public Command feedStop() {
        return new InstantCommand(() -> shooter.feedStop(), shooter);
    }
}
