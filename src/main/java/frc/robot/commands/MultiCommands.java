package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.subsystems.shooter.Shooter;

public class MultiCommands {
    private ShooterCommands shooterCommands;
    private LEDCommands ledCommands;
    private Shooter shooter;

    public MultiCommands(ShooterCommands shooterCommands, LEDCommands ledCommands, Shooter shooter) {
        this.shooterCommands = shooterCommands;
        this.ledCommands = ledCommands;
        this.shooter = shooter;
    }

    public Command shoot() {
        if (ledCommands != null) {
            return shooterCommands
                    .setFeedSpeed(() -> shooter.atTargetSpeed() ? Shooter.Constants.feedSpeed : 0)
                    .alongWith(
                            shooterCommands.flyShoot(),
                            ledCommands.setColor(() ->
                                    shooter.atTargetSpeed() ? new RGBWColor(0, 255, 0) : new RGBWColor(255, 0, 0)))
                    .withName("shoot");
        } else {
            return shooterCommands
                    .setFeedSpeed(() -> shooter.atTargetSpeed() ? Shooter.Constants.feedSpeed : 0)
                    .alongWith(shooterCommands.flyShoot())
                    .withName("shoot");
        }
    }

    public Command shootStop() {
        return shooterCommands.setFeedSpeed(() -> 0).alongWith(shooterCommands.setFlySpeed(() -> 0));
    }
}
