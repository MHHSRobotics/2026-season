package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.io.GyroIO;

public class GyroSim extends SubsystemBase {
    private GyroIO gyro;
    private SwerveSim swerveSim;

    public GyroSim(GyroIO gyroIO, SwerveSim swerveSim) {
        gyro = gyroIO;
        this.swerveSim = swerveSim;
    }

    @Override
    public void periodic() {
        gyro.setMechYaw(swerveSim.getPhysicalPose().getRotation().getZ());
        gyro.setMechPitch(swerveSim.getPhysicalPose().getRotation().getY());
        gyro.setMechRoll(swerveSim.getPhysicalPose().getRotation().getX());
        gyro.setMechYawVelocity(0);
        gyro.setMechPitchVelocity(0);
        gyro.setMechRollVelocity(0);
    }
}
