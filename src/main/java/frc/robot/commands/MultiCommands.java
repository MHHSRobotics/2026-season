package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class MultiCommands {
    private ShooterCommands shooterCommands;
    private SwerveCommands swerveCommands;
    private LEDCommands ledCommands;
    private Shooter shooter;
    private Swerve swerve;

    public MultiCommands(
            ShooterCommands shooterCommands,
            SwerveCommands swerveCommands,
            LEDCommands ledCommands,
            Shooter shooter,
            Swerve swerve) {
        this.shooterCommands = shooterCommands;
        this.swerveCommands = swerveCommands;
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

    // Gets the shooter's field position, accounting for its offset from robot center
    private Translation2d getShooterFieldPos() {
        Pose2d pose = swerve.getPose();
        Translation2d rotatedOffset = Shooter.Constants.shooterPos.rotateBy(pose.getRotation());
        return pose.getTranslation().plus(rotatedOffset);
    }

    // Gets the field-relative velocity of the shooter (includes rotational contribution from omega)
    private Translation2d getShooterFieldVelocity() {
        Pose2d pose = swerve.getPose();
        ChassisSpeeds robotSpeeds = swerve.getChassisSpeeds();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, pose.getRotation());

        // Velocity from chassis translation
        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;

        // Add velocity contribution from robot rotation at the shooter offset
        // v = omega x r (cross product in 2D: omega * (-ry, rx) in field frame)
        Translation2d rotatedOffset = Shooter.Constants.shooterPos.rotateBy(pose.getRotation());
        vx += -fieldSpeeds.omegaRadiansPerSecond * rotatedOffset.getY();
        vy += fieldSpeeds.omegaRadiansPerSecond * rotatedOffset.getX();

        return new Translation2d(vx, vy);
    }

    // Computes radial/lateral velocity decomposition and returns {vRadial, vLateral, dist}
    private double[] getShooterDecomposition() {
        Translation2d shooterPos = getShooterFieldPos();
        Translation2d hubPos = Swerve.Constants.hubPosition.get().getTranslation();
        Translation2d shooterVel = getShooterFieldVelocity();

        Translation2d toHub = hubPos.minus(shooterPos);
        double dist = toHub.getNorm();
        if (dist < 0.01) return new double[] {0, 0, dist};
        Translation2d hubDir = toHub.div(dist);

        // Radial (toward hub) and lateral (perpendicular) velocity components
        double vRadial = shooterVel.getX() * hubDir.getX() + shooterVel.getY() * hubDir.getY();
        Translation2d lateralDir = new Translation2d(-hubDir.getY(), hubDir.getX());
        double vLateral = shooterVel.getX() * lateralDir.getX() + shooterVel.getY() * lateralDir.getY();

        return new double[] {vRadial, vLateral, dist};
    }

    // Gets the compensated aim angle (blue-origin radians) to cancel lateral ball drift
    private double getCompensatedAimAngle() {
        Translation2d shooterPos = getShooterFieldPos();
        Translation2d hubPos = Swerve.Constants.hubPosition.get().getTranslation();

        double[] decomp = getShooterDecomposition();
        double vRadial = decomp[0];
        double vLateral = decomp[1];
        double dist = decomp[2];

        if (dist < 0.01) return Math.atan2(hubPos.getY() - shooterPos.getY(), hubPos.getX() - shooterPos.getX());

        // Conversion factor: rad/s → horizontal exit velocity (m/s)
        double convFactor =
                Shooter.Constants.flywheelRadius * Shooter.Constants.efficiency * Math.cos(Shooter.Constants.exitAngle);

        // Approximate time of flight: distance / compensated horizontal ball speed
        double compensatedHorizontalVel = getShooterSpeed(dist) * convFactor - vRadial;
        if (compensatedHorizontalVel < 0.1) compensatedHorizontalVel = 0.1;
        double timeOfFlight = dist / compensatedHorizontalVel;

        // Offset aim point opposite to lateral drift
        Translation2d toHub = hubPos.minus(shooterPos);
        Translation2d hubDir = toHub.div(dist);
        Translation2d lateralDir = new Translation2d(-hubDir.getY(), hubDir.getX());
        Translation2d aimPoint = hubPos.minus(lateralDir.times(vLateral * timeOfFlight));

        return Math.atan2(aimPoint.getY() - shooterPos.getY(), aimPoint.getX() - shooterPos.getX());
    }

    // Shoots with auto distance calibration, motion compensation, and hub alignment
    public Command shoot() {
        if (Constants.swerveEnabled && Constants.visionEnabled) {
            return shootAtSpeed(() -> {
                        double[] decomp = getShooterDecomposition();
                        double vRadial = decomp[0];
                        double dist = decomp[2];

                        // Compensate flywheel speed for radial motion
                        double convFactor = Shooter.Constants.flywheelRadius
                                * Shooter.Constants.efficiency
                                * Math.cos(Shooter.Constants.exitAngle);
                        double targetHorizontalVel = getShooterSpeed(dist) * convFactor;
                        double compensatedVel = targetHorizontalVel - vRadial;

                        return compensatedVel / convFactor;
                    })
                    .alongWith(swerveCommands.setRotationTarget(this::getCompensatedAimAngle))
                    .withName("shoot with compensation");
        } else {
            return shootDefault();
        }
    }
}
