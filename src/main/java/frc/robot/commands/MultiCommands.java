package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

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
                return getShooterSpeed(swerve.getDistanceFromHub());
            });
        } else {
            return shootDefault();
        }
    }

    public Command shootWithHinge() {
        if (Constants.intakeEnabled && Constants.swerveEnabled) {
            return shoot().alongWith(
                            new RepeatCommand(intakeCommands.switchHinge().andThen(new WaitCommand(0.75))));
        } else {
            return shoot();
        }
    }

    public Command getSingleAuto(String pathName, boolean flipped) {
        return intakeCommands
                .intake()
                .alongWith(swerveCommands
                        .getTrajCommand(pathName, flipped)
                        .andThen(shootWithHinge().alongWith(swerveCommands.moveToTrajEnd(pathName, flipped))));
    }

    public Command getDoubleAuto(String pathName1, boolean flipped1, String pathName2, boolean flipped2) {
        return intakeCommands
                .intake()
                .alongWith(swerveCommands
                        .getTrajCommand(pathName1, flipped1)
                        .andThen(shootWithHinge().alongWith(swerveCommands.moveToTrajEnd(pathName1, flipped1)).withTimeout(5))
                        .andThen(swerveCommands.getTrajCommand(pathName2, flipped2))
                        .andThen(shootWithHinge().alongWith(swerveCommands.moveToTrajEnd(pathName2, flipped2))));
    }
}
