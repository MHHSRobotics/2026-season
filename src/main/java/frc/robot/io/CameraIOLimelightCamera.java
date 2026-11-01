package frc.robot.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.Logger;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.swerve.Swerve;

public class CameraIOLimelightCamera extends CameraIO {
    // NetworkTables name of the Limelight, e.g. "limelight" or "limelight-front".
    private final String limelightName;
    private final Transform3d robotToCamera;
    private boolean limelightRecorded = false;
    private final int missedHBTolerance = 10;

    private double currentHB = -1;

    public CameraIOLimelightCamera(
            String name, String logPath, String limelightName, Transform3d robotToCamera, int pipelineIndex) {
        super(name, logPath);
        this.limelightName = limelightName;
        this.robotToCamera = robotToCamera;

        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
        // Tell the Limelight where it sits on the robot so its bot-pose is correct.
        LimelightHelpers.setCameraPose_RobotSpace(
                limelightName,
                robotToCamera.getX(),
                robotToCamera.getY(),
                robotToCamera.getZ(),
                Math.toDegrees(robotToCamera.getRotation().getX()),
                Math.toDegrees(robotToCamera.getRotation().getY()),
                Math.toDegrees(robotToCamera.getRotation().getZ()));
    }

    public boolean isConnected() {
        double HB = LimelightHelpers.getHeartbeat(limelightName);
        boolean alive = (HB - currentHB) < missedHBTolerance;

        currentHB = HB;
        Logger.recordOutput("LIMELIGHT/HB", HB);

        return alive;
    }

    private void logLimelightRecording() {

        if (!limelightRecorded && DriverStation.getMatchTime() < 1) {
            limelightRecorded = true;

            LimelightHelpers.setRewindEnabled(limelightName, true);
            LimelightHelpers.triggerRewindCapture(limelightName, 165);
        }
    }

    public String getLimelightName() {
        return limelightName;
    }

    @Override
    public void update() {
        // A tag count > 0 is our "connected + has data" signal.

        inputs.connected = isConnected();

        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        boolean valid = estimate != null && estimate.tagCount > 0;
        // inputs.connected = valid;

        if (valid && estimate.tagCount <= Swerve.VisionConstants.maxMeasurements) {
            // Limelight already gives a field-relative robot pose (Pose2d).
            inputs.poses = new Pose3d[] {new Pose3d(estimate.pose)};
            inputs.poseTimestamps = new double[] {estimate.timestampSeconds};
            inputs.ambiguities =
                    new double[] {estimate.rawFiducials.length > 0 ? estimate.rawFiducials[0].ambiguity : 0.0};
            inputs.tagCounts = new int[] {estimate.tagCount};
        } else {
            inputs.poses = new Pose3d[0];
            inputs.poseTimestamps = new double[0];
            inputs.ambiguities = new double[0];
            inputs.tagCounts = new int[0];
        }

        super.update();

        Logger.recordOutput(getLogPath() + "/Transform", robotToCamera);
    }
}
