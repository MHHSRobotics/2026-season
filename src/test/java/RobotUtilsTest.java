import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.util.Field;
import frc.robot.util.RobotUtils;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for RobotUtils field mirroring math.
 *
 * The field is currently configured with C2 (rotational) symmetry,
 * so invert() rotates 180 degrees around the field center.
 */
public class RobotUtilsTest {

    private static final double EPSILON = 1e-9;
    private static final double FL = Field.fieldLength; // 16.54
    private static final double FW = Field.fieldWidth; // 8.07

    // --- Pose2d invert (C2 symmetry) ---

    @Test
    void invertPose2dOriginRotatesToOppositeCorner() {
        Pose2d pose = new Pose2d(0, 0, Rotation2d.kZero);
        Pose2d inverted = RobotUtils.invert(pose);

        assertEquals(FL, inverted.getX(), EPSILON);
        assertEquals(FW, inverted.getY(), EPSILON);
        assertEquals(Math.PI, inverted.getRotation().getRadians(), EPSILON);
    }

    @Test
    void invertPose2dFieldCenterStaysAtCenter() {
        Pose2d pose = new Pose2d(FL / 2, FW / 2, Rotation2d.kZero);
        Pose2d inverted = RobotUtils.invert(pose);

        assertEquals(FL / 2, inverted.getX(), EPSILON);
        assertEquals(FW / 2, inverted.getY(), EPSILON);
        assertEquals(Math.PI, inverted.getRotation().getRadians(), EPSILON);
    }

    @Test
    void invertPose2dDoubleInvertReturnsOriginal() {
        Pose2d pose = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(45));
        Pose2d doubleInverted = RobotUtils.invert(RobotUtils.invert(pose));

        assertEquals(pose.getX(), doubleInverted.getX(), EPSILON);
        assertEquals(pose.getY(), doubleInverted.getY(), EPSILON);
        assertEquals(
                pose.getRotation().getRadians(), doubleInverted.getRotation().getRadians(), EPSILON);
    }

    @Test
    void invertPose2dWithRotation() {
        // Facing right (0 rad) should become facing left (pi rad)
        Pose2d pose = new Pose2d(1.0, 1.0, Rotation2d.kZero);
        Pose2d inverted = RobotUtils.invert(pose);

        assertEquals(FL - 1.0, inverted.getX(), EPSILON);
        assertEquals(FW - 1.0, inverted.getY(), EPSILON);
        assertEquals(Math.PI, inverted.getRotation().getRadians(), EPSILON);
    }

    // --- Translation2d invert (C2 symmetry) ---

    @Test
    void invertTranslation2dOrigin() {
        Translation2d trans = new Translation2d(0, 0);
        Translation2d inverted = RobotUtils.invert(trans);

        assertEquals(FL, inverted.getX(), EPSILON);
        assertEquals(FW, inverted.getY(), EPSILON);
    }

    @Test
    void invertTranslation2dFieldCenter() {
        Translation2d trans = new Translation2d(FL / 2, FW / 2);
        Translation2d inverted = RobotUtils.invert(trans);

        assertEquals(FL / 2, inverted.getX(), EPSILON);
        assertEquals(FW / 2, inverted.getY(), EPSILON);
    }

    @Test
    void invertTranslation2dDoubleInvertReturnsOriginal() {
        Translation2d trans = new Translation2d(5.0, 3.0);
        Translation2d doubleInverted = RobotUtils.invert(RobotUtils.invert(trans));

        assertEquals(trans.getX(), doubleInverted.getX(), EPSILON);
        assertEquals(trans.getY(), doubleInverted.getY(), EPSILON);
    }

    // --- Rotation2d invert (C2 symmetry) ---

    @Test
    void invertRotation2dZeroBecomesPI() {
        Rotation2d rot = Rotation2d.kZero;
        Rotation2d inverted = RobotUtils.invert(rot);

        assertEquals(Math.PI, inverted.getRadians(), EPSILON);
    }

    @Test
    void invertRotation2dDoubleInvertReturnsOriginal() {
        Rotation2d rot = Rotation2d.fromDegrees(30);
        Rotation2d doubleInverted = RobotUtils.invert(RobotUtils.invert(rot));

        assertEquals(rot.getRadians(), doubleInverted.getRadians(), EPSILON);
    }

    // --- Pose3d invert (C2 symmetry) ---

    @Test
    void invertPose3dFlipsXYKeepsZ() {
        Pose3d pose = new Pose3d(1.0, 2.0, 0.5, new Rotation3d(0, 0, 0));
        Pose3d inverted = RobotUtils.invert(pose);

        assertEquals(FL - 1.0, inverted.getX(), EPSILON);
        assertEquals(FW - 2.0, inverted.getY(), EPSILON);
        assertEquals(0.5, inverted.getZ(), EPSILON);
    }

    @Test
    void invertPose3dDoubleInvertReturnsOriginal() {
        Pose3d pose = new Pose3d(3.0, 2.0, 1.0, new Rotation3d(0, 0, Math.PI / 4));
        Pose3d doubleInverted = RobotUtils.invert(RobotUtils.invert(pose));

        assertEquals(pose.getX(), doubleInverted.getX(), EPSILON);
        assertEquals(pose.getY(), doubleInverted.getY(), EPSILON);
        assertEquals(pose.getZ(), doubleInverted.getZ(), EPSILON);
    }

    // --- Translation3d invert (C2 symmetry) ---

    @Test
    void invertTranslation3dFlipsXYKeepsZ() {
        Translation3d trans = new Translation3d(1.0, 2.0, 3.0);
        Translation3d inverted = RobotUtils.invert(trans);

        assertEquals(FL - 1.0, inverted.getX(), EPSILON);
        assertEquals(FW - 2.0, inverted.getY(), EPSILON);
        assertEquals(3.0, inverted.getZ(), EPSILON);
    }

    @Test
    void invertTranslation3dDoubleInvertReturnsOriginal() {
        Translation3d trans = new Translation3d(5.0, 3.0, 1.5);
        Translation3d doubleInverted = RobotUtils.invert(RobotUtils.invert(trans));

        assertEquals(trans.getX(), doubleInverted.getX(), EPSILON);
        assertEquals(trans.getY(), doubleInverted.getY(), EPSILON);
        assertEquals(trans.getZ(), doubleInverted.getZ(), EPSILON);
    }
}
