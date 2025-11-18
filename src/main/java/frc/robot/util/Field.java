package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import org.littletonrobotics.junction.Logger;

public class Field {
    // Field dimensions
    public static final double fieldLength = 17.55;
    public static final double fieldWidth = 8.05;

    // Whether the field this season has rotational (C2) or reflected (D2) symmetry
    public enum FieldSymmetry {
        C2,
        D2
    }

    public static final FieldSymmetry symm = FieldSymmetry.C2;

    // Get the april tag field layout for the current season
    public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final FieldPose2d reefPose = new FieldPose2d(4.4958, 4.03, Rotation2d.kZero);

    public static final double armToCenter = 0.1919; // Meters from center of bot to arm line

    public static final double branchDist = 0.33; // Distance between two branches on the same side (meters)

    public static final double reefRadius = 0.832; // Distance from center of the reef to center of an edge (meters)

    public static final double botRadius = 0.4318; // Distance from center of bot to edge of the bot (meters)

    public static final double alignGap = 0.25; // Extra gap between the bot and reef

    public static FieldPose2d[][] scoringPoses =
            new FieldPose2d[6][2]; // Scoring poses as a 2D array, first reef side, then branch

    // Calculate all scoring poses around the reef
    static {
        Pose2d[] visualization = new Pose2d[12];

        // Calculate the robot's approach distance
        double approachDist = botRadius + reefRadius + alignGap;

        Pose2d blueReefPose = reefPose.getOnBlue();
        for (int side = 0; side < 6; side++) {
            // Calculate the angle for this side of the hexagon (60 degrees per side)
            double sideAngle = side * Math.PI / 3.0;
            Rotation2d sideRotation = new Rotation2d(sideAngle);

            Pose2d rotatedPose = blueReefPose.transformBy(new Transform2d(0, 0, sideRotation));

            for (int branch = 0; branch < 2; branch++) {
                // Figure out the offset along the reef edge for this branch
                // Branch 0 is offset by -branchDist/2, branch 1 is offset by +branchDist/2
                double branchOffset = (branch - 0.5) * branchDist;

                // Make the final scoring pose by:
                // 1. Start at reef center (reefPose)
                // 2. Rotate to face this side of the reef
                // 3. Move forward to the scoring position (distance from reef center to robot position)
                // 4. Shift sideways to align with the branch
                Pose2d scoringPose = rotatedPose.transformBy(
                        new Transform2d(approachDist, armToCenter + branchOffset, Rotation2d.kZero));

                scoringPoses[side][branch] = new FieldPose2d(scoringPose);

                visualization[side * 2 + branch] = scoringPose;
            }
        }
        Logger.recordOutput("Field/ReefPoses", visualization);
    }
}
