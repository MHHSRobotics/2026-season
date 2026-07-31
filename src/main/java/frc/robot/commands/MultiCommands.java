package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.Field;

public class MultiCommands {
    public static class Constants {
        public static final double hingeTime = 0.75;
        public static final double shootTime = 5;
    }

    // A complete shot solution for one loop cycle: where to aim, how fast to spin, and how
    // fast the heading target is moving.
    private static class ShotSolution {
        public final Translation2d virtualTarget;
        public final double distance;
        public final double shooterSpeed;
        public final double botAngle;
        public final double omegaFeedforward;
        public final double flightTime;

        public ShotSolution(
                Translation2d virtualTarget,
                double distance,
                double shooterSpeed,
                double botAngle,
                double omegaFeedforward,
                double flightTime) {
            this.virtualTarget = virtualTarget;
            this.distance = distance;
            this.shooterSpeed = shooterSpeed;
            this.botAngle = botAngle;
            this.omegaFeedforward = omegaFeedforward;
            this.flightTime = flightTime;
        }
    }

    // Shooter position relative to robot center (meters, robot-frame: +X = forward, +Y = left)
    private static final Translation2d shooterOffset = new Translation2d(-0.3048, 0.0);

    private static final double gravity = 9.80665;

    // Virtual target solve limits. Convergence is fast when the correction is small, but a
    // close-range shot with a large correction walks outward over several passes (a 2 m shot
    // at 3 m/s tangential settles at ~6 m), so iterate to a tolerance rather than a fixed
    // count. The arithmetic is trivial next to the rest of the loop.
    private static final int maxSolveIterations = 8;
    private static final double solveToleranceMeters = 0.005;

    // Time constant for smoothing the measured chassis velocity. The velocity comes from
    // module states and is noisy enough to visibly jitter the aim target unfiltered.
    private static final double velocityFilterTau = 0.06;

    // Shot geometry. The shooter has a fixed hood, so these three numbers fully determine
    // flight time (see getFlightTime) and no exit-speed estimate is needed anywhere.
    private static final LoggedNetworkNumber hoodAngleDeg = new LoggedNetworkNumber("Shooter/HoodAngleDeg", 73.0);
    private static final LoggedNetworkNumber exitHeight = new LoggedNetworkNumber("Shooter/ExitHeightMeters", 0.4318);
    private static final LoggedNetworkNumber targetHeight =
            new LoggedNetworkNumber("Shooter/TargetHeightMeters", 1.825);

    // Time between the pose/velocity estimate this solve reads and the ball actually leaving
    // the shooter. Used to push the pose forward so we aim from where we will be.
    private static final LoggedNetworkNumber aimLatency = new LoggedNetworkNumber("Shooter/AimLatencySec", 0.04);

    // Scales the tangential velocity the shooter picks up from the robot spinning about its
    // center. Defaults off: the term is real physics, but it closes a positive feedback loop.
    // Spinning at omega moves the shooter sideways at shooterOffset * omega, which shifts the
    // virtual target sideways by shooterOffset * omega * flightTime, which the theta
    // controller answers with more omega. At 4 m with a 1.5 s flight and rotationKP = 5 the
    // loop gain is roughly
    //     0.3048 * 1.5 / 4 * 5  +  0.3048 / 4  =  0.65
    // which does not diverge on its own but amplifies disturbances ~3x, and wrapped in the
    // loop's phase lag (velocityFilterTau, steer response, pose latency) it rings. It is also
    // worth little: as the aim converges omega goes to zero and so does the correction. Set
    // to 1 only if you want it back and have confirmed the aim is well damped without it.
    private static final LoggedNetworkNumber offsetVelScale = new LoggedNetworkNumber("Shooter/OffsetVelScale", 0.0);

    // Robot speed above which the heading correction gets large enough that a moving shot is
    // not worth attempting. Tangential motion is what drives this; radial motion is nearly free.
    private static final LoggedNetworkNumber maxShootingSpeed =
            new LoggedNetworkNumber("Shooter/MaxShootingSpeed", 2.0);

    private static final LoggedNetworkNumber aimToleranceRad =
            new LoggedNetworkNumber("Shooter/AimToleranceRad", Math.toRadians(2.0));

    private ShooterCommands shooterCommands;
    private IntakeCommands intakeCommands;
    private SwerveCommands swerveCommands;
    private Swerve swerve;

    private final LinearFilter velFilterX = LinearFilter.singlePoleIIR(velocityFilterTau, frc.robot.Constants.loopTime);
    private final LinearFilter velFilterY = LinearFilter.singlePoleIIR(velocityFilterTau, frc.robot.Constants.loopTime);

    // The solve is shared by the aim target, the aim feedforward, the flywheel setpoint and
    // the readiness check, so cache it per loop cycle instead of recomputing it four times.
    private ShotSolution cachedSolution;
    private long cachedSolutionTimestamp = -1;

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
        return 32.64 * dist + 219.9;
    }

    // Gets how long the ball is in the air for a shot at the given distance.
    //
    // The hood is fixed, so a shot that lands in the hub has to satisfy both
    //   horizontal:  v * cos(hood) * t = distance
    //   vertical:    exitHeight + v * sin(hood) * t - 1/2 * g * t^2 = targetHeight
    // and substituting the first into the second cancels the launch speed entirely:
    //   exitHeight + distance * tan(hood) - targetHeight = 1/2 * g * t^2
    // So flight time follows from geometry alone - it needs no estimate of how fast the
    // flywheel throws the ball, which is the one number we do not reliably know.
    //
    // Distance is clamped to the same range as getShooterSpeed, since outside that range the
    // flywheel setpoint is clamped too and the shot is not calibrated anyway.
    private double getFlightTime(double dist) {
        dist = MathUtil.clamp(dist, 1, 7);
        double rise = exitHeight.get() + dist * Math.tan(Math.toRadians(hoodAngleDeg.get())) - targetHeight.get();
        if (rise <= 0) {
            return 0;
        }
        return Math.sqrt(2 * rise / gravity);
    }

    // Solves the moving shot for this loop cycle using the virtual target method.
    private ShotSolution solve() {
        Pose2d pose = swerve.getPose();
        ChassisSpeeds fieldSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getChassisSpeeds(), pose.getRotation());

        // Velocity of the shooter itself: chassis velocity plus the tangential term the
        // shooter picks up from the robot rotating about its center, since it sits behind it.
        Translation2d fieldOffset = shooterOffset.rotateBy(pose.getRotation());
        double omega = fieldSpeeds.omegaRadiansPerSecond * offsetVelScale.get();
        Translation2d rawVel = new Translation2d(
                fieldSpeeds.vxMetersPerSecond - omega * fieldOffset.getY(),
                fieldSpeeds.vyMetersPerSecond + omega * fieldOffset.getX());
        Translation2d shooterVel =
                new Translation2d(velFilterX.calculate(rawVel.getX()), velFilterY.calculate(rawVel.getY()));

        // Distances stay measured from the robot center so the existing getShooterSpeed
        // regression, which was tuned against swerve.getDistanceFromHub(), still applies. The
        // shooter offset points straight back along the robot's X axis, so once we are aimed
        // it lies on the line to the hub and contributes no bearing error either way.
        Translation2d shooterPos = pose.getTranslation().plus(shooterVel.times(aimLatency.get()));
        Translation2d hub = Field.hubPosition.get().getTranslation();

        // The robot's velocity is horizontal, so it cannot change the vertical component of
        // the launch and therefore cannot change the flight time. All it does is carry the
        // ball sideways during that flight, which means aiming and ranging at
        //   virtualTarget = hub - shooterVelocity * flightTime
        // is exact for a fixed hood. Flight time depends on the virtual distance, so iterate.
        Translation2d virtualTarget = hub;
        double distance = shooterPos.getDistance(hub);
        double flightTime = 0;
        int iterations = 0;
        for (int i = 0; i < maxSolveIterations; i++) {
            iterations = i + 1;
            double previousDistance = distance;
            flightTime = getFlightTime(distance);
            virtualTarget = hub.minus(shooterVel.times(flightTime));
            distance = shooterPos.getDistance(virtualTarget);
            if (Math.abs(distance - previousDistance) < solveToleranceMeters) {
                break;
            }
        }

        Translation2d delta = virtualTarget.minus(shooterPos);
        double normSq = delta.getX() * delta.getX() + delta.getY() * delta.getY();
        double botAngle = normSq < 1e-6 ? swerve.getRotation().getRadians() : Math.atan2(delta.getY(), delta.getX());

        // d/dt of atan2(delta) with the virtual target held fixed, which works out to the
        // tangential component of our own velocity over the range. Feeding this forward is
        // what removes the tracking lag a pure P controller leaves while translating.
        double omegaFeedforward =
                normSq < 1e-6 ? 0 : (shooterVel.getX() * delta.getY() - shooterVel.getY() * delta.getX()) / normSq;

        ShotSolution solution = new ShotSolution(
                virtualTarget, distance, getShooterSpeed(distance), botAngle, omegaFeedforward, flightTime);

        Logger.recordOutput("Shooter/DistanceToHub", shooterPos.getDistance(hub));
        Logger.recordOutput("Shooter/VirtualDistance", solution.distance);
        Logger.recordOutput("Shooter/SolveIterations", iterations);
        Logger.recordOutput("Shooter/FlightTime", solution.flightTime);
        Logger.recordOutput(
                "Shooter/VirtualTarget", new Pose2d(solution.virtualTarget, Rotation2d.fromRadians(solution.botAngle)));
        Logger.recordOutput("Shooter/ShooterFieldVelX", shooterVel.getX());
        Logger.recordOutput("Shooter/ShooterFieldVelY", shooterVel.getY());
        Logger.recordOutput("Shooter/BaseSpeed", getShooterSpeed(shooterPos.getDistance(hub)));
        Logger.recordOutput("Shooter/CompensatedSpeed", solution.shooterSpeed);
        Logger.recordOutput("Shooter/BaseAngle", getAngleToHub());
        Logger.recordOutput("Shooter/CompAngleToHub", solution.botAngle);
        Logger.recordOutput("Shooter/AngleDifference", MathUtil.angleModulus(solution.botAngle - getAngleToHub()));
        Logger.recordOutput("Shooter/AimFeedforward", solution.omegaFeedforward);

        return solution;
    }

    // Gets this loop cycle's shot solution, solving it once and reusing it.
    private ShotSolution getSolution() {
        long timestamp = Logger.getTimestamp();
        if (cachedSolution == null || timestamp != cachedSolutionTimestamp) {
            cachedSolutionTimestamp = timestamp;
            cachedSolution = solve();
        }
        return cachedSolution;
    }

    // Gets the bearing from the robot to the hub, ignoring motion.
    // Field.hubPosition.get() is already flipped for our alliance, so the bearing it produces
    // is already in the right frame and must not be inverted again.
    private double getAngleToHub() {
        Translation2d hubPos = Field.hubPosition.get().getTranslation();
        Pose2d pose = swerve.getPose();
        return Math.atan2(hubPos.getY() - pose.getY(), hubPos.getX() - pose.getX());
    }

    public boolean isAimedAndSpunUp() {
        double targetAngle =
                frc.robot.Constants.shooterVelocityCompensationEnabled ? getSolution().botAngle : getAngleToHub();
        double angleError = Math.abs(MathUtil.angleModulus(swerve.getRotation().getRadians() - targetAngle));

        ChassisSpeeds speeds = swerve.getChassisSpeeds();
        double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        boolean slowEnough = speed < maxShootingSpeed.get();

        boolean ready = angleError < aimToleranceRad.get() && slowEnough && shooterCommands.atTargetSpeed();

        Logger.recordOutput("Shooter/AimErrorRad", angleError);
        Logger.recordOutput("Shooter/RobotSpeed", speed);
        Logger.recordOutput("Shooter/SlowEnoughToShoot", slowEnough);
        Logger.recordOutput("Shooter/ReadyToShoot", ready);

        return ready;
    }

    // Aims the robot at the hub with velocity compensation
    public Command aimAtHub() {
        if (!frc.robot.Constants.swerveEnabled) {
            return swerveCommands.setRotationOutput(() -> 0);
        }
        if (frc.robot.Constants.shooterVelocityCompensationEnabled) {
            return swerveCommands.setRotationTarget(() -> getSolution().botAngle, () -> getSolution().omegaFeedforward);
        }
        return swerveCommands.aimAt(Field.hubPosition);
    }

    // Shoots with auto distance calibration and velocity compensation
    public Command shoot() {
        if (!frc.robot.Constants.swerveEnabled) {
            return shootDefault();
        }
        if (frc.robot.Constants.shooterVelocityCompensationEnabled) {
            // A moving shot has to aim itself: the heading target moves with the robot, so
            // the aim has to run for as long as the shot does. Aiming lives on its own
            // pseudo-subsystem, so this composes with the driver's translation - they keep
            // driving, this owns rotation.
            //
            // Keep the flywheel tracking the moving setpoint continuously, but hold the feed
            // until the drivetrain has actually converged, otherwise balls leave the shooter
            // while it is still slewing onto the compensated heading.
            //
            // Interruption stays at the default kCancelSelf so nudging the turn stick still
            // schedules steer() and takes rotation back. kCancelIncoming here would lock the
            // driver out of turning for as long as the trigger is held.
            return aimAtHub()
                    .alongWith(
                            shooterCommands.setFlySpeed(() -> getSolution().shooterSpeed),
                            shooterCommands.setFeedSpeed(() -> isAimedAndSpunUp() ? Shooter.Constants.feedSpeed : 0))
                    .withName("compensated shoot");
        }
        return shootAtSpeed(() -> getShooterSpeed(swerve.getDistanceFromHub()));
    }

    // Shoots at the ranged setpoint without taking over rotation or waiting to be aimed.
    // For auto, where the trajectory and moveToTrajEnd already own the heading: aiming here
    // would both fight them for it and, since they require swerveRotation too, make the
    // parallel composition illegal. Velocity compensation still applies to the setpoint.
    public Command shootRanged() {
        if (!frc.robot.Constants.swerveEnabled) {
            return shootDefault();
        }
        if (frc.robot.Constants.shooterVelocityCompensationEnabled) {
            return shootAtSpeed(() -> getSolution().shooterSpeed);
        }
        return shootAtSpeed(() -> getShooterSpeed(swerve.getDistanceFromHub()));
    }

    @SuppressWarnings("unused")
    public Command shootWithHinge() {
        if (frc.robot.Constants.intakeEnabled && frc.robot.Constants.swerveEnabled) {
            return shootRanged()
                    .alongWith(new RepeatCommand(
                            intakeCommands.switchHinge().andThen(new WaitCommand(Constants.hingeTime))));
        } else {
            return shoot();
        }
    }

    public Command getSingleAuto(String pathName, boolean flipped) {
        return intakeCommands
                .intake()
                .alongWith(
                        Commands.waitSeconds(1).andThen(intakeCommands.setHingeDown()),
                        swerveCommands.resetToTrajStart(pathName, flipped),
                        swerveCommands
                                .getTrajCommand(pathName, flipped)
                                .andThen(Commands.waitSeconds(0.5)
                                        .andThen(shootWithHinge())
                                        .alongWith(swerveCommands.moveToTrajEnd(pathName, flipped))));
    }

    public Command getDoubleAuto(String pathName1, boolean flipped1, String pathName2, boolean flipped2) {
        return intakeCommands
                .intake()
                .alongWith(
                        Commands.waitSeconds(1).andThen(intakeCommands.setHingeDown()),
                        swerveCommands.resetToTrajStart(pathName1, flipped1),
                        swerveCommands
                                .getTrajCommand(pathName1, flipped1)
                                .andThen(Commands.waitSeconds(0.5)
                                        .andThen(shootWithHinge())
                                        .alongWith(swerveCommands.moveToTrajEnd(pathName1, flipped1))
                                        .withTimeout(Constants.shootTime))
                                .andThen(swerveCommands
                                        .getTrajCommand(pathName2, flipped2)
                                        .alongWith(intakeCommands.setHingeDown()))
                                .andThen(Commands.waitSeconds(0.5)
                                        .andThen(shootWithHinge())
                                        .alongWith(swerveCommands.moveToTrajEnd(pathName2, flipped2))));
    }
}
