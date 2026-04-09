package frc.robot.commands;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.littletonrobotics.junction.Logger;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveRotation;
import frc.robot.subsystems.swerve.SwerveTranslation;
import frc.robot.util.Alerts;
import frc.robot.util.FieldPose2d;
import frc.robot.util.FieldTranslation2d;
import frc.robot.util.RobotUtils;

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
                                    Swerve.Constants.movePow.get());
                            double angle = Math.atan2(y, x);
                            double sign = RobotUtils.onRedAlliance() ? -1 : 1;
                            swerve.setTranslation(
                                    sign * scale * Math.cos(angle) * Swerve.Constants.maxLinearSpeedMetersPerSec,
                                    sign * scale * Math.sin(angle) * Swerve.Constants.maxLinearSpeedMetersPerSec,
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
                                    Swerve.Constants.turnPow.get());
                            rotation = Math.copySign(rotationScale, rotation);

                            swerve.setRotation(rotation * Swerve.Constants.maxAngularSpeedRadPerSec);
                        },
                        swerveRotation)
                .finallyDo(() -> swerve.setRotation(0))
                .withName("swerve steer");
    }

    // Sets raw translation output (m/s), for auto/test use
    public Command setPositionOutput(DoubleSupplier dx, DoubleSupplier dy) {
        return Commands.run(() -> swerve.setTranslation(dx.getAsDouble(), dy.getAsDouble(), false), swerveTranslation)
                .finallyDo(() -> swerve.setTranslation(0, 0, false))
                .withName("swerve set position output");
    }

    // Sets raw rotation output (rad/s), for auto/test use
    public Command setRotationOutput(DoubleSupplier omega) {
        return Commands.run(() -> swerve.setRotation(omega.getAsDouble()), swerveRotation)
                .finallyDo(() -> swerve.setRotation(0))
                .withName("swerve set rotation output");
    }

    // Sets translational and rotational speed
    public Command setSpeed(DoubleSupplier dx, DoubleSupplier dy, DoubleSupplier omega) {
        return Commands.parallel(setPositionOutput(dx, dy), setRotationOutput(omega))
                .withName("swerve set speed");
    }

    public Command setPositionTarget(FieldTranslation2d target) {
        return setPositionTarget(() -> target.get(), () -> Pair.of(0., 0.));
    }

    public Command setPositionTarget(Translation2d target) {
        return setPositionTarget(() -> target, () -> Pair.of(0., 0.));
    }

    public Command setPositionTarget(Supplier<Translation2d> target) {
        return setPositionTarget(target, () -> Pair.of(0., 0.));
    }
    // PID-controlled translation to a field position
    public Command setPositionTarget(Supplier<Translation2d> target, Supplier<Pair<Double, Double>> feedforwards) {
        return Commands.run(
                        () -> {
                            Translation2d t = target.get();
                            double xOutput = swerve.getXController()
                                    .calculate(swerve.getPose().getX(), t.getX());
                            double yOutput = swerve.getYController()
                                    .calculate(swerve.getPose().getY(), t.getY());
                            Pair<Double, Double> feedforward = feedforwards.get();
                            swerve.setTranslation(
                                    xOutput + feedforward.getFirst(), yOutput + feedforward.getSecond(), true);
                            swerve.setPIDPosition(true);
                        },
                        swerveTranslation)
                .finallyDo(() -> {
                    swerve.setTranslation(0, 0, false);
                    swerve.setPIDPosition(false);
                })
                .withName("swerve set position target");
    }

    public Command setRotationTarget(double theta) {
        return setRotationTarget(() -> theta, () -> 0);
    }

    public Command setRotationTarget(DoubleSupplier theta) {
        return setRotationTarget(theta, () -> 0);
    }

    // PID-controlled rotation to a field heading (blue-origin radians)
    public Command setRotationTarget(DoubleSupplier theta, DoubleSupplier feedforward) {
        return Commands.run(
                        () -> {
                            double output = swerve.getThetaController()
                                    .calculate(swerve.getPose().getRotation().getRadians(), theta.getAsDouble());
                            swerve.setRotation(output + feedforward.getAsDouble());
                            swerve.setPIDRotation(true);
                        },
                        swerveRotation)
                .finallyDo(() -> {
                    swerve.setRotation(0);
                    swerve.setPIDRotation(false);
                })
                .withName("swerve set rotation target");
    }

    // public static class FollowTraj extends Command {
    //     private double startTime;
    //     private Trajectory<SwerveSample> traj;
    //     private Swerve swerve;

    //     public FollowTraj(Trajectory<SwerveSample> traj, Swerve swerve) {
    //         this.traj = traj;
    //         this.swerve = swerve;
    //         addRequirements(swerve);
    //     }

    //     @Override
    //     public void initialize() {
    //         startTime = RobotUtils.getTime();
    //     }

    //     @Override
    //     public void execute() {
    //         SwerveSample trajSample1 = traj.sampleAt(RobotUtils.getTime() - startTime, RobotUtils.onRedAlliance())
    //                 .get();
    //         SwerveSample trajSample2 = traj.sampleAt(
    //                         (RobotUtils.getTime() - startTime) + 0.1, RobotUtils.onRedAlliance())
    //                 .get();
    //         Pose2d currentPose = swerve.getPose();
    //         Pose2d targetPose = trajSample1.getPose();
    //         double xOutput = swerve.getXController().calculate(currentPose.getX(), targetPose.getX()) +
    // trajSample2.vx;
    //         double yOutput = swerve.getYController().calculate(currentPose.getY(), targetPose.getY()) +
    // trajSample2.vy;
    //         double thetaOutput = swerve.getThetaController()
    //                         .calculate(
    //                                 currentPose.getRotation().getRadians(),
    //                                 targetPose.getRotation().getRadians())
    //                 + trajSample2.omega;
    //         swerve.setTranslation(xOutput, yOutput, true);
    //         swerve.setRotation(thetaOutput);
    //         swerve.setPIDPosition(true);
    //         swerve.setPIDRotation(true);
    //     }

    //     @Override
    //     public void end(boolean e) {
    //         swerve.setTranslation(0, 0, true);
    //         swerve.setRotation(0);
    //         swerve.setPIDPosition(false);
    //         swerve.setPIDRotation(false);
    //     }

    //     @Override
    //     public boolean isFinished() {
    //         return RobotUtils.getTime() - startTime > traj.getTotalTime();
    //     }
    // }

    public static class FollowTraj extends Command {
        private static final double STEER_LOOKAHEAD = 0.1;
        private static final double POSITION_TOLERANCE = 0.1;

        private List<SwerveSample> samples;
        private final Trajectory<SwerveSample> traj;
        private final Swerve swerve;

        private int currentSegment;
        private double tCurrent;
        private double lastTickTime;

        public FollowTraj(Trajectory<SwerveSample> traj, Swerve swerve) {
            this.traj = traj;
            this.swerve = swerve;
            addRequirements(swerve);
        }

        @Override
        public void initialize() {
            samples = traj.samples();

            tCurrent = 0.0;
            currentSegment = 0;
            lastTickTime = RobotUtils.getTime();

            swerve.setPIDPosition(true);
            swerve.setPIDRotation(true);
        }

        @Override
        public void execute() {
            double now = RobotUtils.getTime();
            double dt = now - lastTickTime;
            lastTickTime = now;

            Pose2d currentPose = swerve.getPose();
            Translation2d robotPos = currentPose.getTranslation();

            // Find closest straight-line segment in the search window.
            int bestSeg = currentSegment;
            int searchSeg = currentSegment;
            double bestDistSq = Double.MAX_VALUE;
            while (samples.get(searchSeg).t < tCurrent + dt * 2.0 && searchSeg < samples.size() - 1) {
                SwerveSample a = maybeFlip(samples.get(searchSeg));
                SwerveSample b = maybeFlip(samples.get(searchSeg + 1));
                double d = distanceSqToSegment(robotPos, a.x, a.y, b.x, b.y);
                if (d < bestDistSq) {
                    bestDistSq = d;
                    bestSeg = searchSeg;
                }
                searchSeg++;
            }

            // Analytic projection onto the chosen segment.
            SwerveSample segStart = maybeFlip(samples.get(bestSeg));
            SwerveSample segEnd = maybeFlip(samples.get(bestSeg + 1));
            double dx = segEnd.x - segStart.x;
            double dy = segEnd.y - segStart.y;
            double lenSq = dx * dx + dy * dy;
            double segT;
            if (lenSq < 1e-12) {
                segT = 0.0;
            } else {
                segT = ((robotPos.getX() - segStart.x) * dx + (robotPos.getY() - segStart.y) * dy) / lenSq;
                segT = MathUtil.clamp(segT, 0, 1);
            }

            // Resolve tCurrent from the segment parameter, with monotonicity enforcement.
            double newTCurrent = segStart.t + segT * (segEnd.t - segStart.t);
            if (newTCurrent < tCurrent) {
                newTCurrent = tCurrent;
            }
            tCurrent = newTCurrent;
            currentSegment = bestSeg;

            SwerveSample pidTargetPose =
                    traj.sampleAt(tCurrent, RobotUtils.onRedAlliance()).get();
            SwerveSample feedforwardPose = traj.sampleAt(
                            Math.min(tCurrent + STEER_LOOKAHEAD, traj.getTotalTime()), RobotUtils.onRedAlliance())
                    .get();

            // PID feedback on the projected point, feedforward velocities from the lookahead.
            double xOutput =
                    swerve.getXController().calculate(currentPose.getX(), pidTargetPose.x) + feedforwardPose.vx;
            double yOutput =
                    swerve.getYController().calculate(currentPose.getY(), pidTargetPose.y) + feedforwardPose.vy;
            double thetaOutput = swerve.getThetaController()
                            .calculate(currentPose.getRotation().getRadians(), pidTargetPose.heading)
                    + feedforwardPose.omega;

            swerve.setTranslation(xOutput, yOutput, true);
            swerve.setRotation(thetaOutput);

            Logger.recordOutput("Auto/CurrentSegment", currentSegment);
            Logger.recordOutput("Auto/CurrentPathTime", tCurrent);
        }

        @Override
        public void end(boolean interrupted) {
            swerve.setTranslation(0, 0, true);
            swerve.setRotation(0);
            swerve.setPIDPosition(false);
            swerve.setPIDRotation(false);
        }

        @Override
        public boolean isFinished() {

            if (tCurrent < traj.getTotalTime() - 1e-3) {
                return false;
            }
            Pose2d finalPose = traj.getFinalPose(RobotUtils.onRedAlliance()).get();
            double dx = swerve.getPose().getX() - finalPose.getX();
            double dy = swerve.getPose().getY() - finalPose.getY();
            return Math.hypot(dx, dy) < POSITION_TOLERANCE;
        }

        // --- helpers ---

        private SwerveSample maybeFlip(SwerveSample s) {
            return RobotUtils.onRedAlliance() ? s.flipped() : s;
        }

        private double distanceSqToSegment(Translation2d p, double ax, double ay, double bx, double by) {
            double dx = bx - ax;
            double dy = by - ay;
            double lenSq = dx * dx + dy * dy;
            if (lenSq < 1e-12) {
                double ex = p.getX() - ax;
                double ey = p.getY() - ay;
                return ex * ex + ey * ey;
            }
            double t = ((p.getX() - ax) * dx + (p.getY() - ay) * dy) / lenSq;
            t = Math.max(0.0, Math.min(1.0, t));
            double projX = ax + t * dx;
            double projY = ay + t * dy;
            double ex = p.getX() - projX;
            double ey = p.getY() - projY;
            return ex * ex + ey * ey;
        }
    }

    public Command followTraj(Trajectory<SwerveSample> traj) {
        return new FollowTraj(traj, swerve);
    }

    public Command getTrajCommand(String name, boolean flipped) {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(name);
        if (traj.isEmpty()) {
            Alerts.create("No trajectory named " + name + " could be found", AlertType.kError);
            return Commands.none();
        }
        Trajectory<SwerveSample> flippedTraj = flipped ? traj.get().mirrorY() : traj.get();
        return followTraj(flippedTraj);
    }

    public Command moveToTrajEnd(String name, boolean flipped) {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(name);
        if (traj.isEmpty()) {
            Alerts.create("No trajectory named " + name + " could be found", AlertType.kError);
            return Commands.none();
        }
        Trajectory<SwerveSample> realTraj = traj.get();
        Trajectory<SwerveSample> flippedTraj = flipped ? realTraj.mirrorY() : realTraj;
        return setPoseTarget(
                () -> flippedTraj.getFinalPose(RobotUtils.onRedAlliance()).get());
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
                            swerve.setPIDRotation(true);
                        },
                        swerveRotation)
                .finallyDo(() -> {
                    swerve.setRotation(0);
                    swerve.setPIDRotation(false);
                })
                .withName("swerve aim at");
    }

    // PID-controlled drive to a field pose
    public Command setPoseTarget(FieldPose2d pose) {
        return setPoseTarget(() -> pose.get());
    }

    // PID-controlled drive to a field pose
    public Command setPoseTarget(Pose2d pose) {
        return setPoseTarget(() -> pose);
    }

    // PID-controlled drive to a field pose
    public Command setPoseTarget(Supplier<Pose2d> pose) {
        return Commands.parallel(
                        setPositionTarget(() -> pose.get().getTranslation()),
                        setRotationTarget(() -> pose.get().getRotation().getRadians()))
                .withName("swerve set pose target");
    }

    // Puts the swerve drive into an X position so it can't be pushed
    public Command lock() {
        return new InstantCommand(() -> swerve.lock(), swerveTranslation, swerveRotation).withName("swerve lock");
    }

    // Stops all swerve output
    public Command stop() {
        return new InstantCommand(() -> swerve.stop(), swerveTranslation, swerveRotation).withName("swerve stop");
    }

    // Reset swerve gyro to 0
    public Command resetGyro() {
        return new InstantCommand(() -> swerve.resetGyro()).withName("reset gyro");
    }

    // Reset swerve pose
    public Command resetPose(Pose2d pose) {
        return resetPose(() -> pose);
    }

    // Reset swerve pose
    public Command resetPose(Supplier<Pose2d> pose) {
        return new InstantCommand(() -> swerve.resetPose(pose.get())).withName("reset pose");
    }

    public Command resetToTrajStart(String name, boolean flipped) {
        Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(name);
        if (traj.isEmpty()) {
            Alerts.create("No trajectory named " + name + " could be found", AlertType.kError);
            return Commands.none();
        }
        Trajectory<SwerveSample> realTraj = traj.get();
        Trajectory<SwerveSample> flippedTraj = flipped ? realTraj.mirrorY() : realTraj;
        return resetPose(
                () -> flippedTraj.getInitialPose(RobotUtils.onRedAlliance()).get());
    }
}
