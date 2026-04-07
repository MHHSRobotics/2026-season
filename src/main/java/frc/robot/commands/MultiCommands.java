package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.Field;
import frc.robot.util.RobotUtils;

public class MultiCommands {
    public static class Constants {
        public static final double hingeTime = 0.75;
        public static final double shootTime = 5;
    }

    private static class ShotVector {
        public final Translation2d robotRelativeLaunchVelocity;
        public final double shooterSpeed;
        public final double botAngleBlue;

        ShotVector(Translation2d robotRelativeLaunchVelocity, double shooterSpeed, double botAngleBlue) {
            this.robotRelativeLaunchVelocity = robotRelativeLaunchVelocity;
            this.shooterSpeed = shooterSpeed;
            this.botAngleBlue = botAngleBlue;
        }
    }

    // Shooter position relative to robot center (meters, robot-frame: +X = forward, +Y = left)
    private static final Translation2d shooterOffset = new Translation2d(-0.3048, 0.0);

    // Converts shooter speed units into estimated projectile exit speed in m/s.
    private static final LoggedNetworkNumber launchSpeedPerShooterSpeed =
            new LoggedNetworkNumber("Shooter/LaunchSpeedPerShooterSpeed", 0.0071);
    private static final double minLaunchSpeedMetersPerSecond = 0.1;
    private static final LoggedNetworkNumber aimToleranceRad =
            new LoggedNetworkNumber("Shooter/AimToleranceRad", Math.toRadians(2.0));

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

    private double getEstimatedLaunchSpeed(double shooterSpeed) {
        return Math.max(minLaunchSpeedMetersPerSecond, shooterSpeed * launchSpeedPerShooterSpeed.get());
    }

    // Gets the field-relative position of the shooter (not bot center)
    private Translation2d getShooterFieldPosition() {
        Pose2d pose = swerve.getPose();
        // Rotate the robot-relative shooter offset by the robot's heading, then add to robot position
        return pose.getTranslation().plus(shooterOffset.rotateBy(pose.getRotation()));
    }

    // Gets the field-relative velocity of the shooter (accounts for rotation around bot center)
    private Translation2d getShooterFieldVelocity() {
        Pose2d pose = swerve.getPose();
        ChassisSpeeds speeds = swerve.getChassisSpeeds();
        // Convert robot-relative linear velocity to field-relative
        double cosHeading = pose.getRotation().getCos();
        double sinHeading = pose.getRotation().getSin();
        double fieldVx = speeds.vxMetersPerSecond * cosHeading - speeds.vyMetersPerSecond * sinHeading;
        double fieldVy = speeds.vxMetersPerSecond * sinHeading + speeds.vyMetersPerSecond * cosHeading;
        // Add tangential velocity from bot rotation at the shooter offset
        // v_tangential = omega x r (cross product in 2D: omega * (-ry, rx) rotated to field frame)
        Translation2d fieldOffset = shooterOffset.rotateBy(pose.getRotation());
        double tangentialVx = -speeds.omegaRadiansPerSecond * fieldOffset.getY();
        double tangentialVy = speeds.omegaRadiansPerSecond * fieldOffset.getX();
        return new Translation2d(fieldVx + tangentialVx, fieldVy + tangentialVy);
    }

    private ShotVector getShotVector(Translation2d shooterPos, Translation2d shooterVel) {
        Translation2d hubPos = Field.hubPosition.get().getTranslation();
        Translation2d toHub = hubPos.minus(shooterPos);
        double dist = toHub.getNorm();
        if (dist < 0.01) {
            return new ShotVector(
                    new Translation2d(),
                    Shooter.Constants.defaultSpeed.get(),
                    swerve.getRotation().getRadians());
        }

        double baseShooterSpeed = getShooterSpeed(dist);
        double baseLaunchSpeed = getEstimatedLaunchSpeed(baseShooterSpeed);
        Translation2d desiredFieldVelocity = toHub.div(dist).times(baseLaunchSpeed);
        Translation2d robotRelativeLaunchVelocity = desiredFieldVelocity.minus(shooterVel);
        double compensatedLaunchSpeed = Math.max(minLaunchSpeedMetersPerSecond, robotRelativeLaunchVelocity.getNorm());
        double compensatedShooterSpeed = compensatedLaunchSpeed / Math.max(1e-6, launchSpeedPerShooterSpeed.get());
        double allianceAngle = Math.atan2(robotRelativeLaunchVelocity.getY(), robotRelativeLaunchVelocity.getX());
        double blueAngle = RobotUtils.invertThetaToAlliance(allianceAngle);

        Logger.recordOutput("Shooter/DistanceToHub", dist);
        Logger.recordOutput("Shooter/BaseSpeed", baseShooterSpeed);
        Logger.recordOutput("Shooter/BaseLaunchSpeed", baseLaunchSpeed);
        Logger.recordOutput("Shooter/DesiredFieldVelX", desiredFieldVelocity.getX());
        Logger.recordOutput("Shooter/DesiredFieldVelY", desiredFieldVelocity.getY());
        Logger.recordOutput("Shooter/ShooterFieldVelX", shooterVel.getX());
        Logger.recordOutput("Shooter/ShooterFieldVelY", shooterVel.getY());
        Logger.recordOutput("Shooter/RobotRelativeLaunchVelX", robotRelativeLaunchVelocity.getX());
        Logger.recordOutput("Shooter/RobotRelativeLaunchVelY", robotRelativeLaunchVelocity.getY());
        Logger.recordOutput("Shooter/CompensatedLaunchSpeed", compensatedLaunchSpeed);
        Logger.recordOutput("Shooter/CompensatedSpeed", compensatedShooterSpeed);
        Logger.recordOutput("Shooter/CompAngleToHub", Math.toDegrees(allianceAngle));

        return new ShotVector(robotRelativeLaunchVelocity, compensatedShooterSpeed, blueAngle);
    }

    private ShotVector getCurrentShotVector() {
        return getShotVector(getShooterFieldPosition(), getShooterFieldVelocity());
    }

    private double getAngleToHub() {
        Translation2d hubPos = Field.hubPosition.get().getTranslation();
        Translation2d shooterPos = getShooterFieldPosition();
        double allianceAngle = Math.atan2(hubPos.getY() - shooterPos.getY(), hubPos.getX() - shooterPos.getX());
        return RobotUtils.invertThetaToAlliance(allianceAngle);
    }

    // Computes the angle from the shooter to the hub using the full launch-vector solve.
    // Returns angle in blue-origin coordinates (suitable for setRotationTarget).
    private double getCompensatedAngleToHub() {
        return getCurrentShotVector().botAngleBlue;
    }

    // Computes the angular feedforward (rad/s) from the time derivative of the solved bot angle.
    private double getAimFeedforward() {
        Translation2d shooterPos = getShooterFieldPosition();
        Translation2d shooterVel = getShooterFieldVelocity();
        ShotVector currentShot = getShotVector(shooterPos, shooterVel);
        Translation2d nextShooterPos = shooterPos.plus(shooterVel.times(frc.robot.Constants.loopTime));
        ShotVector nextShot = getShotVector(nextShooterPos, shooterVel);
        double dAngleDt =
                MathUtil.angleModulus(nextShot.botAngleBlue - currentShot.botAngleBlue) / frc.robot.Constants.loopTime;

        Logger.recordOutput("Shooter/AimFeedforward", dAngleDt);

        return dAngleDt;
    }

    // Gets the effective distance from the shooter to the hub (for speed calculation)
    private double getShooterDistanceFromHub() {
        Translation2d hubPos = Field.hubPosition.get().getTranslation();
        return hubPos.getDistance(getShooterFieldPosition());
    }

    // Gets velocity-compensated shooter speed (rot/s).
    // Model:
    // - Distance tuning gives the stationary flywheel setpoint for this shot.
    // - A tunable linear conversion estimates projectile exit speed from that setpoint.
    // - Shooter XY velocity is subtracted from the desired field-relative launch vector.
    // - The resulting robot-relative launch vector sets both bot angle and shooter speed.
    private double getCompensatedShooterSpeed() {
        return getCurrentShotVector().shooterSpeed;
    }

    public boolean isAimedAndSpunUp() {
        double targetAngle =
                frc.robot.Constants.shooterVelocityCompensationEnabled ? getCompensatedAngleToHub() : getAngleToHub();
        double angleError = Math.abs(MathUtil.angleModulus(swerve.getRotation().getRadians() - targetAngle));
        boolean ready = angleError < aimToleranceRad.get() && shooterCommands.atTargetSpeed();

        Logger.recordOutput("Shooter/AimErrorRad", angleError);
        Logger.recordOutput("Shooter/ReadyToShoot", ready);

        return ready;
    }

    // Aims the robot at the hub with velocity compensation
    public Command aimAtHub() {
        if (!frc.robot.Constants.swerveEnabled) {
            return swerveCommands.setRotationOutput(() -> 0);
        }
        if (frc.robot.Constants.shooterVelocityCompensationEnabled) {
            return swerveCommands.setRotationTarget(() -> getCompensatedAngleToHub(), () -> getAimFeedforward());
        }
        return swerveCommands.aimAt(Field.hubPosition);
    }

    // Shoots with auto distance calibration and radial velocity compensation
    public Command shoot() {
        if (frc.robot.Constants.swerveEnabled) {
            if (frc.robot.Constants.shooterVelocityCompensationEnabled) {
                return shootAtSpeed(() -> getCompensatedShooterSpeed());
            }
            return shootAtSpeed(() -> getShooterSpeed(getShooterDistanceFromHub()));
        } else {
            return shootDefault();
        }
    }

    @SuppressWarnings("unused")
    public Command shootWithHinge() {
        if (frc.robot.Constants.intakeEnabled && frc.robot.Constants.swerveEnabled) {
            return shoot().alongWith(new RepeatCommand(
                    intakeCommands.switchHinge().andThen(new WaitCommand(Constants.hingeTime))));
        } else {
            return shoot();
        }
    }

    public Command getSingleAuto(String pathName, boolean flipped) {
        return intakeCommands
                .intake()
                .alongWith(
                        intakeCommands.setHingeDown(),
                        swerveCommands.resetToTrajStart(pathName, flipped),
                        swerveCommands
                                .getTrajCommand(pathName, flipped)
                                .andThen(shootWithHinge().alongWith(swerveCommands.moveToTrajEnd(pathName, flipped))));
    }

    public Command getDoubleAuto(String pathName1, boolean flipped1, String pathName2, boolean flipped2) {
        return intakeCommands
                .intake()
                .alongWith(
                        intakeCommands.setHingeDown(),
                        swerveCommands.resetToTrajStart(pathName1, flipped1),
                        swerveCommands
                                .getTrajCommand(pathName1, flipped1)
                                .andThen(shootWithHinge()
                                        .alongWith(swerveCommands.moveToTrajEnd(pathName1, flipped1))
                                        .withTimeout(Constants.shootTime))
                                .andThen(swerveCommands
                                        .getTrajCommand(pathName2, flipped2)
                                        .alongWith(intakeCommands.setHingeDown()))
                                .andThen(
                                        shootWithHinge().alongWith(swerveCommands.moveToTrajEnd(pathName2, flipped2))));
    }
}
