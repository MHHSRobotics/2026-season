package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;

public class SimpleSwerveSim extends SwerveSim {
    private SwerveDriveKinematics kinematics;

    private SwerveModuleSim[] moduleSims;

    public SimpleSwerveSim(SwerveModuleSim[] moduleSims) {
        this.moduleSims = moduleSims;
        kinematics = new SwerveDriveKinematics(Swerve.getModuleTranslations());
    }

    @Override
    public void periodic() {
        Pose2d currentPose2d=currentPose.toPose2d();
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = moduleSims[i].getState();

            states[i].angle = states[i].angle.plus(Rotation2d.fromRotations(
                    (Math.random() * 2 - 1) * Swerve.Constants.simSwerveError)); // Random angle error for simulation
            states[i].speedMetersPerSecond *= (1
                    + (Math.random() * 2 - 1) * Swerve.Constants.simSwerveError); // Random speed error for simulation
        }
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(states);
        Twist2d twist = speeds.toTwist2d(Constants.loopTime);
        currentPose2d = currentPose2d.exp(twist);
        currentPose=new Pose3d(currentPose2d);
        Logger.recordOutput("SwerveSim/ActualPose", currentPose);
    }
}
