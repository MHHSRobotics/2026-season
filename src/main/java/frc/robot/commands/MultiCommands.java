package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class MultiCommands {
    private ShooterCommands shooterCommands;
    private Swerve swerve;

    public MultiCommands(ShooterCommands shooterCommands, Swerve swerve) {
        this.shooterCommands = shooterCommands;
        this.swerve = swerve;
    }

    public Command shootAtSpeed(DoubleSupplier speed) {
        return shooterCommands.shoot(speed).withName("shoot");
    }

    public Command shootStop() {
        return shooterCommands.setFeedSpeed(() -> 0).alongWith(shooterCommands.setFlySpeed(() -> 0));
    }

    // Shoots at a default speed for feeding
    public Command shootDefault() {
        return shootAtSpeed(() -> Shooter.Constants.defaultSpeed.get());
    }

    // Gets target shooter speed from distance
    private double getShooterSpeed(double dist) {
        // Clamp equation from 1 to 7 meters
        dist = MathUtil.clamp(dist, 1, 7);
        return 4.7143 * dist * dist - 3.119 * dist + 298.92;
    }

    // Shoots with auto distance calibration
    public Command shoot() {
        if (Constants.swerveEnabled && Constants.visionEnabled) {
            return shootAtSpeed(() -> {
                // System.out.println(getShooterSpeed(swerve.getDistanceFromHub()));
                return getShooterSpeed(swerve.getDistanceFromHub());
            });
        } else {
            return shootDefault();
        }
    }
}
