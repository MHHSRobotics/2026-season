package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.subsystems.intake.Intake;
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
        public final double shooterSpeed;
        public final double botAngle;

        public ShotVector(double shooterSpeed, double botAngle) {
            this.shooterSpeed = shooterSpeed;
            this.botAngle = botAngle;
        }
    }

    // Shooter position relative to robot center (meters, robot-frame: +X = forward, +Y = left)
    private static final Translation2d shooterOffset = new Translation2d(-0.3048, 0.0);

    // Converts shooter speed units into estimated projectile exit speed in m/s.
    private static final LoggedNetworkNumber launchSpeedPerShooterSpeed =
            new LoggedNetworkNumber("Shooter/LaunchSpeedPerShooterSpeed", 0.0071);
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
        if (shooterCommands == null) {
            return Commands.none();
        }
        return shooterCommands.shoot(speed).withName("shoot");
    }

    public Command shootStop() {
        if (shooterCommands == null) {
            return Commands.none();
        }
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

    private double getEstimatedLaunchSpeed(double shooterSpeed) {
        return shooterSpeed * launchSpeedPerShooterSpeed.get();
    }

    // Gets the field-relative position of the shooter (not bot center)
    private Translation2d getShooterFieldPosition() {
        Pose2d pose = swerve.getPose();
        // Rotate the robot-relative shooter offset by the robot's heading, then add to robot position
        return pose.getTranslation();
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

    private ShotVector getShotVector(Translation2d botPos, Translation2d shooterVel) {
        Translation2d hubPos = Field.hubPosition.get().getTranslation();
        Translation2d toHub = hubPos.minus(botPos);
        double dist = toHub.getNorm();
        if (dist < 0.01) {
            return new ShotVector(
                    Shooter.Constants.defaultSpeed.get(), swerve.getRotation().getRadians());
        }

        double baseShooterSpeed = getShooterSpeed(dist);
        double baseLaunchSpeed = getEstimatedLaunchSpeed(baseShooterSpeed);
        Translation2d desiredFieldVelocity = toHub.div(dist).times(baseLaunchSpeed);
        Translation2d robotRelativeLaunchVelocity = desiredFieldVelocity.minus(shooterVel);
        double compensatedLaunchSpeed = robotRelativeLaunchVelocity.getNorm();
        double compensatedShooterSpeed = compensatedLaunchSpeed / launchSpeedPerShooterSpeed.get();
        double targetAngle = Math.atan2(robotRelativeLaunchVelocity.getY(), robotRelativeLaunchVelocity.getX());
        double baseAngle = getAngleToHub();

        Logger.recordOutput("Shooter/DistanceToHub", dist);
        Logger.recordOutput("Shooter/BaseSpeed", baseShooterSpeed);
        Logger.recordOutput("Shooter/BaseLaunchSpeed", baseLaunchSpeed);
        Logger.recordOutput("Shooter/BaseAngle", baseAngle);
        Logger.recordOutput("Shooter/DesiredFieldVelX", desiredFieldVelocity.getX());
        Logger.recordOutput("Shooter/DesiredFieldVelY", desiredFieldVelocity.getY());
        Logger.recordOutput("Shooter/ShooterFieldVelX", shooterVel.getX());
        Logger.recordOutput("Shooter/ShooterFieldVelY", shooterVel.getY());
        Logger.recordOutput("Shooter/RobotRelativeLaunchVelX", robotRelativeLaunchVelocity.getX());
        Logger.recordOutput("Shooter/RobotRelativeLaunchVelY", robotRelativeLaunchVelocity.getY());
        Logger.recordOutput("Shooter/CompensatedLaunchSpeed", compensatedLaunchSpeed);
        Logger.recordOutput("Shooter/CompensatedSpeed", compensatedShooterSpeed);
        Logger.recordOutput("Shooter/SpeedDifference", compensatedShooterSpeed - baseShooterSpeed);
        Logger.recordOutput("Shooter/CompAngleToHub", targetAngle);
        Logger.recordOutput("Shooter/AngleDifference", targetAngle - baseAngle);

        return new ShotVector(compensatedShooterSpeed, targetAngle);
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
        return getCurrentShotVector().botAngle;
    }

    // Computes the angular feedforward (rad/s) from the time derivative of the solved bot angle.
    private double getAimFeedforward() {
        Translation2d shooterPos = getShooterFieldPosition();
        Translation2d shooterVel = getShooterFieldVelocity();
        ShotVector currentShot = getShotVector(shooterPos, shooterVel);
        Translation2d nextShooterPos = shooterPos.plus(shooterVel.times(frc.robot.Constants.loopTime));
        ShotVector nextShot = getShotVector(nextShooterPos, shooterVel);
        double dAngleDt =
                MathUtil.angleModulus(nextShot.botAngle - currentShot.botAngle) / frc.robot.Constants.loopTime;

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
        if (swerveCommands == null) {
            return Commands.none();
        }
        if (frc.robot.Constants.shooterVelocityCompensationEnabled) {
            return swerveCommands.setRotationTarget(() -> getCompensatedAngleToHub(), () -> getAimFeedforward());
        }
        return swerveCommands.aimAt(Field.hubPosition);
    }

    // Shoots with auto distance calibration and radial velocity compensation
    public Command shoot() {
        if (swerveCommands == null) {
            return shootDefault();
        }
        if (frc.robot.Constants.shooterVelocityCompensationEnabled) {
            return shootAtSpeed(() -> getCompensatedShooterSpeed());
        }
        return shootAtSpeed(() -> getShooterSpeed(swerve.getDistanceFromHub()));
    }

    public Command shootWithHinge() {
        if (intakeCommands == null) {
            return shoot();
        }
        return shoot().alongWith(
                        new RepeatCommand(intakeCommands.switchHinge().andThen(new WaitCommand(Constants.hingeTime))));
    }

    public Command intakeWithSpeed() {
        if (intakeCommands == null) {
            return Commands.none();
        }
        if (swerve == null) {
            return intakeCommands.setRollerTargetSpeed(() -> Intake.Constants.minSpeed.get());
        }
        return intakeCommands.setRollerTargetSpeed(() -> Intake.Constants.minSpeed.get()
                + Math.max(0, swerve.getChassisSpeeds().vxMetersPerSecond) / Intake.Constants.rollerRadius);
    }

    public Command getSingleAuto(String pathName, boolean flipped) {
        if (intakeCommands == null || swerveCommands == null) {
            return Commands.none();
        }
        return intakeWithSpeed()
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
        if (intakeCommands == null || swerveCommands == null) {
            return Commands.none();
        }
        return intakeWithSpeed()
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
