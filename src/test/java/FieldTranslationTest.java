import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.util.Field;
import frc.robot.util.FieldTranslation2d;
import frc.robot.util.FieldTranslation3d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/** Tests for FieldTranslation2d and FieldTranslation3d alliance-aware wrappers. */
public class FieldTranslationTest {

    private static final double EPSILON = 1e-9;
    private static final double FL = Field.fieldLength;
    private static final double FW = Field.fieldWidth;

    // --- FieldTranslation2d ---

    @Test
    void fieldTranslation2dGetOnBlueReturnsOriginal() {
        FieldTranslation2d ft = new FieldTranslation2d(5.0, 3.0);
        Translation2d blue = ft.getOnBlue();

        assertEquals(5.0, blue.getX(), EPSILON);
        assertEquals(3.0, blue.getY(), EPSILON);
    }

    @Test
    void fieldTranslation2dGetOnRedInverts() {
        FieldTranslation2d ft = new FieldTranslation2d(1.0, 2.0);
        Translation2d red = ft.getOnRed();

        // C2: both X and Y flip
        assertEquals(FL - 1.0, red.getX(), EPSILON);
        assertEquals(FW - 2.0, red.getY(), EPSILON);
    }

    @Test
    void fieldTranslation2dDefaultConstructorIsOrigin() {
        FieldTranslation2d ft = new FieldTranslation2d();
        Translation2d blue = ft.getOnBlue();

        assertEquals(0, blue.getX(), EPSILON);
        assertEquals(0, blue.getY(), EPSILON);
    }

    // --- FieldTranslation3d ---

    @Test
    void fieldTranslation3dGetOnBlueReturnsOriginal() {
        FieldTranslation3d ft = new FieldTranslation3d(5.0, 3.0, 1.5);
        Translation3d blue = ft.getOnBlue();

        assertEquals(5.0, blue.getX(), EPSILON);
        assertEquals(3.0, blue.getY(), EPSILON);
        assertEquals(1.5, blue.getZ(), EPSILON);
    }

    @Test
    void fieldTranslation3dGetOnRedFlipsXYKeepsZ() {
        FieldTranslation3d ft = new FieldTranslation3d(1.0, 2.0, 3.0);
        Translation3d red = ft.getOnRed();

        assertEquals(FL - 1.0, red.getX(), EPSILON);
        assertEquals(FW - 2.0, red.getY(), EPSILON);
        assertEquals(3.0, red.getZ(), EPSILON);
    }

    @Test
    void fieldTranslation3dFromTranslation3d() {
        Translation3d original = new Translation3d(4.0, 5.0, 6.0);
        FieldTranslation3d ft = new FieldTranslation3d(original);
        Translation3d blue = ft.getOnBlue();

        assertEquals(original.getX(), blue.getX(), EPSILON);
        assertEquals(original.getY(), blue.getY(), EPSILON);
        assertEquals(original.getZ(), blue.getZ(), EPSILON);
    }

    @Test
    void fieldTranslation3dDefaultConstructorIsOrigin() {
        FieldTranslation3d ft = new FieldTranslation3d();
        Translation3d blue = ft.getOnBlue();

        assertEquals(0, blue.getX(), EPSILON);
        assertEquals(0, blue.getY(), EPSILON);
        assertEquals(0, blue.getZ(), EPSILON);
    }
}
