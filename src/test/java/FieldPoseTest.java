import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import frc.robot.util.Field;
import frc.robot.util.FieldPose2d;
import frc.robot.util.FieldPose3d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/** Tests for FieldPose2d and FieldPose3d alliance-aware wrappers. */
public class FieldPoseTest {

    private static final double EPSILON = 1e-9;
    private static final double FL = Field.fieldLength;
    private static final double FW = Field.fieldWidth;

    // --- FieldPose2d ---

    @Test
    void fieldPose2dGetOnBlueReturnsOriginal() {
        FieldPose2d fp = new FieldPose2d(3.0, 2.0, Math.PI / 4);
        Pose2d blue = fp.getOnBlue();

        assertEquals(3.0, blue.getX(), EPSILON);
        assertEquals(2.0, blue.getY(), EPSILON);
        assertEquals(Math.PI / 4, blue.getRotation().getRadians(), EPSILON);
    }

    @Test
    void fieldPose2dGetOnRedInverts() {
        FieldPose2d fp = new FieldPose2d(1.0, 1.0, 0);
        Pose2d red = fp.getOnRed();

        // C2: 180 degree rotation around field center
        assertEquals(FL - 1.0, red.getX(), EPSILON);
        assertEquals(FW - 1.0, red.getY(), EPSILON);
        assertEquals(Math.PI, red.getRotation().getRadians(), EPSILON);
    }

    @Test
    void fieldPose2dDefaultConstructorIsOrigin() {
        FieldPose2d fp = new FieldPose2d();
        Pose2d blue = fp.getOnBlue();

        assertEquals(0, blue.getX(), EPSILON);
        assertEquals(0, blue.getY(), EPSILON);
        assertEquals(0, blue.getRotation().getRadians(), EPSILON);
    }

    @Test
    void fieldPose2dFromPose2d() {
        Pose2d original = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90));
        FieldPose2d fp = new FieldPose2d(original);
        Pose2d blue = fp.getOnBlue();

        assertEquals(original.getX(), blue.getX(), EPSILON);
        assertEquals(original.getY(), blue.getY(), EPSILON);
        assertEquals(original.getRotation().getRadians(), blue.getRotation().getRadians(), EPSILON);
    }

    // --- FieldPose3d ---

    @Test
    void fieldPose3dGetOnBlueReturnsOriginal() {
        FieldPose3d fp = new FieldPose3d(3.0, 2.0, 1.0, new Rotation3d(0.1, 0.2, 0.3));
        Pose3d blue = fp.getOnBlue();

        assertEquals(3.0, blue.getX(), EPSILON);
        assertEquals(2.0, blue.getY(), EPSILON);
        assertEquals(1.0, blue.getZ(), EPSILON);
    }

    @Test
    void fieldPose3dGetOnRedFlipsXY() {
        FieldPose3d fp = new FieldPose3d(1.0, 2.0, 0.5, new Rotation3d(0, 0, 0));
        Pose3d red = fp.getOnRed();

        assertEquals(FL - 1.0, red.getX(), EPSILON);
        assertEquals(FW - 2.0, red.getY(), EPSILON);
        assertEquals(0.5, red.getZ(), EPSILON);
    }

    @Test
    void fieldPose3dDefaultConstructorIsOrigin() {
        FieldPose3d fp = new FieldPose3d();
        Pose3d blue = fp.getOnBlue();

        assertEquals(0, blue.getX(), EPSILON);
        assertEquals(0, blue.getY(), EPSILON);
        assertEquals(0, blue.getZ(), EPSILON);
    }
}
