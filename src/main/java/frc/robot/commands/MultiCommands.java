package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.Field;

public class MultiCommands {
    private ShooterCommands shooterCommands;
    private IntakeCommands intakeCommands;
    private SwerveCommands swerveCommands;
    private Swerve swerve;

    public MultiCommands(
            ShooterCommands shooterCommands,
            IntakeCommands intakeCommands,
            SwerveCommands swerveCommands,
            Swerve swerve) {
        this.shooterCommands = shooterCommands;
        this.intakeCommands = intakeCommands;
        this.swerveCommands = swerveCommands;
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

    public Command shootWithHinge() {
        if(Constants.intakeEnabled && Constants.swerveEnabled){
            return shoot().alongWith(new RepeatCommand(intakeCommands.switchHinge().andThen(new WaitCommand(0.75))))
                .alongWith(swerveCommands.aimAt(Field.hubPosition));
        }else{
            return shoot();
        }
        
    }
}
