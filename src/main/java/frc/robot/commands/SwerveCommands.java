package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.Field;
import frc.robot.util.FieldPose2d;

public class SwerveCommands {
    private Swerve swerve;

    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
    }

    // Puts the swerve drive into an X position so it can't be pushed
    public Command lock() {
        return new InstantCommand(() -> swerve.lock(), swerve).withName("swerve lock");
    }

    // Reset swerve gyro to 0
    public Command resetGyro() {
        return new InstantCommand(() -> swerve.resetGyro());
    }

    // Drives using the given dx, dy, omega, and field relative inputs. Applies a deadband and scales the values.
    public Command drive(DoubleSupplier dx, DoubleSupplier dy, DoubleSupplier omega, BooleanSupplier fieldRelative) {
        return Commands.run(
                        () -> {
                            double x = dx.getAsDouble();
                            double y = dy.getAsDouble();

                            // Get the distance from (x,y) to the origin
                            double radius = Math.hypot(x, y);

                            // Apply the deadband to the radius and take it to the power of movePow for smoother control
                            double scale = Math.pow(
                                    MathUtil.applyDeadband(radius, Swerve.Constants.moveDeadband),
                                    Swerve.Constants.movePow);

                            // Gets the angle in radians of the line from (0,0) to (x,y)
                            double angle = Math.atan2(y, x);

                            // Get the scaled values of x and y
                            x = scale * Math.cos(angle);
                            y = scale * Math.sin(angle);
                            double rotation = omega.getAsDouble();

                            // Apply the deadband to the absolute value of rotation and take it to the power of turnPow
                            // for
                            // smoother control
                            double rotationScale = Math.pow(
                                    MathUtil.applyDeadband(Math.abs(rotation), Swerve.Constants.turnDeadband),
                                    Swerve.Constants.turnPow);

                            // Copy the sign of rotation to rotationScale to get the final rotation value
                            rotation = Math.copySign(rotationScale, rotation);

                            // Run the swerve drive with the given values of x, y, and rotation
                            swerve.setPositionOutput(
                                    x * Swerve.Constants.maxLinearSpeedMetersPerSec,
                                    y * Swerve.Constants.maxLinearSpeedMetersPerSec);
                            swerve.setRotationOutput(rotation * Swerve.Constants.maxAngularSpeedRadPerSec);
                            swerve.setFieldOriented(fieldRelative.getAsBoolean());
                        },
                        swerve)
                .withName("swerve drive");
    }

    // Command to set x and y speed
    public Command setPositionOutput(double dx, double dy) {
        return new InstantCommand(() -> swerve.setPositionOutput(dx, dy), swerve)
                .withName("swerve set position output");
    }

    // Command to set rotational speed
    public Command setRotationOutput(double omega) {
        return new InstantCommand(() -> swerve.setRotationOutput(omega), swerve).withName("swerve set rotation output");
    }

    // Sets translational and rotational speed
    public Command setSpeed(double dx, double dy, double omega) {
        return setPositionOutput(dx, dy).andThen(setRotationOutput(omega)).withName("swerve set speed");
    }

    // Stops all swerve output
    public Command stop() {
        return setSpeed(0, 0, 0);
    }

    // Command to set position target
    public Command setPositionTarget(double x, double y) {
        return new InstantCommand(() -> swerve.setPositionTarget(x, y), swerve).withName("swerve set position target");
    }

    // Command to set rotation target
    public Command setRotationTarget(double rotation) {
        return new InstantCommand(() -> swerve.setRotationTarget(rotation), swerve)
                .withName("swerve set rotation target");
    }

    // Command to set pose target
    public Command setPoseTarget(FieldPose2d pose) {
        return new InstantCommand(() -> swerve.setPoseTarget(pose), swerve).withName("swerve set pose target");
    }

    // side = 0 aligns to left, side = 1 aligns to right
    public Command alignToSide(int side) {
        return new InstantCommand(
                        () -> {
                            Pose2d pose = swerve.getPose();
                            FieldPose2d closest = null;
                            double closestDist = Double.MAX_VALUE;
                            for (int i = 0; i < Field.scoringPoses.length; i++) {
                                // Get field pose scoring at the given branch
                                FieldPose2d scoringPose = Field.scoringPoses[i][side];
                                // Get the Pose2d that's flipped if on red alliance
                                Pose2d otherPose = scoringPose.get();
                                // Get the distance between bot position and the physical Pose2d
                                double dist = otherPose.getTranslation().getDistance(pose.getTranslation());
                                // If dist is the new closest set the closest pose to scoringPose
                                if (dist < closestDist) {
                                    closestDist = dist;
                                    closest = scoringPose;
                                }
                            }
                            swerve.setPoseTarget(closest);
                        },
                        swerve)
                .withName(side == 0 ? "align to left" : "align to right");
    }
}
