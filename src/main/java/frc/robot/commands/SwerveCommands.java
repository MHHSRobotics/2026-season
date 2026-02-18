package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.swerve.Swerve;
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

    /**
     * Applies polar deadband and power-curve scaling to joystick translation inputs.
     * Returns a two-element array [scaledX, scaledY] in the range [-1, 1].
     */
    public static double[] applyTranslationScaling(double x, double y, double deadband, double power) {
        // Get the distance from (x,y) to the origin
        double radius = Math.hypot(x, y);

        // Apply the deadband to the radius and take it to the power of movePow for smoother control
        double scale = Math.pow(MathUtil.applyDeadband(radius, deadband), power);

        // Gets the angle in radians of the line from (0,0) to (x,y)
        double angle = Math.atan2(y, x);

        // Get the scaled values of x and y
        return new double[] {scale * Math.cos(angle), scale * Math.sin(angle)};
    }

    /**
     * Applies deadband and power-curve scaling to a joystick rotation input.
     * Returns a value in the range [-1, 1] preserving the original sign.
     */
    public static double applyRotationScaling(double rotation, double deadband, double power) {
        double rotationScale = Math.pow(MathUtil.applyDeadband(Math.abs(rotation), deadband), power);
        return Math.copySign(rotationScale, rotation);
    }

    // Drives using the given dx, dy, omega, and field relative inputs. Applies a deadband and scales the values.
    public Command drive(DoubleSupplier dx, DoubleSupplier dy, DoubleSupplier omega, BooleanSupplier fieldRelative) {
        return Commands.run(
                        () -> {
                            double[] scaled = applyTranslationScaling(
                                    dx.getAsDouble(),
                                    dy.getAsDouble(),
                                    Swerve.Constants.moveDeadband,
                                    Swerve.Constants.movePow);

                            double rotation = applyRotationScaling(
                                    omega.getAsDouble(), Swerve.Constants.turnDeadband, Swerve.Constants.turnPow);

                            // Run the swerve drive with the given values of x, y, and rotation
                            swerve.setPositionOutput(
                                    scaled[0] * Swerve.Constants.maxLinearSpeedMetersPerSec,
                                    scaled[1] * Swerve.Constants.maxLinearSpeedMetersPerSec);
                            swerve.setRotationOutput(rotation * Swerve.Constants.maxAngularSpeedRadPerSec);
                            swerve.setFieldOriented(fieldRelative.getAsBoolean());
                        },
                        swerve)
                .ignoringDisable(true)
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
}
