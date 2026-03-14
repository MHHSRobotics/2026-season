package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class MultiCommands {
    private ShooterCommands shooterCommands;
    private LEDCommands ledCommands;
    private Shooter shooter;
    private Swerve swerve;

    public MultiCommands(ShooterCommands shooterCommands, LEDCommands ledCommands, Shooter shooter, Swerve swerve) {
        this.shooterCommands = shooterCommands;
        this.ledCommands = ledCommands;
        this.shooter = shooter;
        this.swerve = swerve;
    }

    public Command shootAtSpeed(DoubleSupplier speed) {
        if (ledCommands != null) {
            return shooterCommands
                    .shoot(speed)
                    .alongWith(ledCommands.setColor(
                            () -> shooter.atTargetSpeed() ? new RGBWColor(0, 255, 0) : new RGBWColor(255, 0, 0)))
                    .withName("shoot");
        } else {
            return shooterCommands.shoot(speed);
        }
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
                double dist=swerve.getDistanceFromHub();
                double goalSpeed=getShooterSpeed(dist);
                double goalXVel=goalSpeed*Shooter.Constants.flywheelRadius*Shooter.Constants.efficiency*Math.cos(Shooter.Constants.exitAngle);
                
                // System.out.println(getShooterSpeed(swerve.getDistanceFromHub()));
                return getShooterSpeed();
            });
        } else {
            return shootDefault();
        }
    }
}
