package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

public class Field {
    // Field dimensions
    public static final double fieldLength = 16.54;
    public static final double fieldWidth = 8.07;

    // Whether the field this season has rotational (C2) or reflected (D2) symmetry
    public enum FieldSymmetry {
        C2,
        D2
    }

    public static final FieldSymmetry symm = FieldSymmetry.C2;

    public static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(
            AprilTagFields.kDefaultField); // Get the april tag field layout for the current season

    // Hub center position in blue alliance coordinates (meters)
    public static final FieldPose2d hubPosition = new FieldPose2d(4.622, 4.035, 0);

    // Outpost bot position in blue alliance coordinates (meters)
    public static final FieldPose2d outpostPosition = new FieldPose2d(0.44, 0.58, Units.degreesToRadians(0));

    // Hang bot position in blue alliance coordinates (meters)
    public static final FieldPose2d hangPosition = new FieldPose2d(1.555, 3.29, Units.degreesToRadians(90));
}
