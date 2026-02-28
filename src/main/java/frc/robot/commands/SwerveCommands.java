package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveRotation;
import frc.robot.subsystems.swerve.SwerveTranslation;
import frc.robot.util.FieldPose2d;
import frc.robot.util.FieldTranslation2d;

public class SwerveCommands {
    private final Swerve swerve;
    private final SwerveTranslation swerveTranslation;
    private final SwerveRotation swerveRotation;

    public SwerveCommands(Swerve swerve, SwerveTranslation swerveTranslation, SwerveRotation swerveRotation) {
        this.swerve = swerve;
        this.swerveTranslation = swerveTranslation;
        this.swerveRotation = swerveRotation;
    }

    // Drives translation using the given dx, dy stick inputs. Applies deadband, power scaling, and max speed.
    public Command drive(DoubleSupplier dx, DoubleSupplier dy, BooleanSupplier fieldCentric) {
        return Commands.run(
                        () -> {
                            double x = dx.getAsDouble();
                            double y = dy.getAsDouble();

                            double radius = Math.hypot(x, y);
                            double scale = Math.pow(
                                    MathUtil.applyDeadband(radius, Swerve.Constants.moveDeadband),
                                    Swerve.Constants.movePow);
                            double angle = Math.atan2(y, x);

                            swerve.setTranslation(
                                    scale * Math.cos(angle) * Swerve.Constants.maxLinearSpeedMetersPerSec,
                                    scale * Math.sin(angle) * Swerve.Constants.maxLinearSpeedMetersPerSec,
                                    fieldCentric.getAsBoolean());
                        },
                        swerveTranslation)
                .finallyDo(() -> swerve.setTranslation(0, 0, false))
                .withName("swerve drive");
    }

    // Steers rotation using the given omega stick input. Applies deadband and power scaling.
    public Command steer(DoubleSupplier omega) {
        return Commands.run(
                        () -> {
                            double rotation = omega.getAsDouble();
                            double rotationScale = Math.pow(
                                    MathUtil.applyDeadband(Math.abs(rotation), Swerve.Constants.turnDeadband),
                                    Swerve.Constants.turnPow);
                            rotation = Math.copySign(rotationScale, rotation);

                            swerve.setRotation(rotation * Swerve.Constants.maxAngularSpeedRadPerSec);
                        },
                        swerveRotation)
                .finallyDo(() -> swerve.setRotation(0))
                .withName("swerve steer");
    }

    // Sets raw translation output (m/s), for auto/test use
    public Command setPositionOutput(double dx, double dy) {
        return Commands.run(() -> swerve.setTranslation(dx, dy, false), swerveTranslation)
                .finallyDo(() -> swerve.setTranslation(0, 0, false))
                .withName("swerve set position output");
    }

    // Sets raw rotation output (rad/s), for auto/test use
    public Command setRotationOutput(double omega) {
        return Commands.run(() -> swerve.setRotation(omega), swerveRotation)
                .finallyDo(() -> swerve.setRotation(0))
                .withName("swerve set rotation output");
    }

    // Sets translational and rotational speed
    public Command setSpeed(double dx, double dy, double omega) {
        return Commands.parallel(setPositionOutput(dx, dy), setRotationOutput(omega))
                .withName("swerve set speed");
    }

    // PID-controlled translation to a field position
    public Command setPositionTarget(FieldTranslation2d target) {
        return Commands.run(
                        () -> {
                            Translation2d t = target.get();
                            double xOutput = swerve.getXController()
                                    .calculate(swerve.getPose().getX(), t.getX());
                            double yOutput = swerve.getYController()
                                    .calculate(swerve.getPose().getY(), t.getY());
                            swerve.setTranslation(xOutput, yOutput, true);
                        },
                        swerveTranslation)
                .finallyDo(() -> swerve.setTranslation(0, 0, false))
                .withName("swerve set position target");
    }

    // PID-controlled rotation to a field heading (blue-origin radians)
    public Command setRotationTarget(double theta) {
        return Commands.run(
                        () -> {
                            Pose2d alliancePose = new FieldPose2d(0, 0, theta).get();
                            double output = swerve.getThetaController()
                                    .calculate(
                                            swerve.getPose().getRotation().getRadians(),
                                            alliancePose.getRotation().getRadians());
                            swerve.setRotation(output);
                        },
                        swerveRotation)
                .finallyDo(() -> swerve.setRotation(0))
                .withName("swerve set rotation target");
    }

    // PID-controlled rotation to aim at a field position (rotates to face the target)
    public Command aimAt(FieldPose2d target) {
        return Commands.run(
                        () -> {
                            Pose2d targetPose = target.get();
                            Pose2d currentPose = swerve.getPose();
                            double angleToTarget = Math.atan2(
                                    targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX());
                            double output = swerve.getThetaController()
                                    .calculate(currentPose.getRotation().getRadians(), angleToTarget);
                            swerve.setRotation(output);
                        },
                        swerveRotation)
                .finallyDo(() -> swerve.setRotation(0))
                .withName("swerve aim at");
    }

    // PID-controlled drive to a field pose
    public Command setPoseTarget(FieldPose2d pose) {
        return Commands.parallel(
                        setPositionTarget(pose.getTranslation()),
                        setRotationTarget(pose.getOnBlue().getRotation().getRadians()))
                .withName("swerve set pose target");
    }

    // Puts the swerve drive into an X position so it can't be pushed
    public Command lock() {
        return new InstantCommand(() -> swerve.lock(), swerveTranslation, swerveRotation)
                .withName("swerve lock");
    }

    // Stops all swerve output
    public Command stop() {
        return new InstantCommand(() -> swerve.stop(), swerveTranslation, swerveRotation)
                .withName("swerve stop");
    }

    // Reset swerve gyro to 0
    public Command resetGyro() {
        return new InstantCommand(() -> swerve.resetGyro()).withName("reset gyro");
    }

    // Reset swerve pose
    public Command resetPose(Pose2d pose) {
        return new InstantCommand(() -> swerve.resetPose(pose)).withName("reset pose");
    }
}
