import frc.robot.commands.SwerveCommands;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/** Tests for the joystick scaling math extracted from SwerveCommands.drive(). */
public class SwerveCommandsTest {

    private static final double EPSILON = 1e-9;
    private static final double DEADBAND = 0.1;
    private static final double POWER = 2.0;

    // --- Translation scaling ---

    @Test
    void translationInsideDeadbandReturnsZero() {
        double[] result = SwerveCommands.applyTranslationScaling(0.05, 0.05, DEADBAND, POWER);
        assertEquals(0, result[0], EPSILON);
        assertEquals(0, result[1], EPSILON);
    }

    @Test
    void translationZeroInputReturnsZero() {
        double[] result = SwerveCommands.applyTranslationScaling(0, 0, DEADBAND, POWER);
        assertEquals(0, result[0], EPSILON);
        assertEquals(0, result[1], EPSILON);
    }

    @Test
    void translationFullForwardReturnsOne() {
        double[] result = SwerveCommands.applyTranslationScaling(1.0, 0, DEADBAND, POWER);
        assertEquals(1.0, result[0], EPSILON);
        assertEquals(0, result[1], EPSILON);
    }

    @Test
    void translationFullLeftReturnsOne() {
        double[] result = SwerveCommands.applyTranslationScaling(0, 1.0, DEADBAND, POWER);
        assertEquals(0, result[0], EPSILON);
        assertEquals(1.0, result[1], EPSILON);
    }

    @Test
    void translationNegativeDirectionWorks() {
        double[] result = SwerveCommands.applyTranslationScaling(-1.0, 0, DEADBAND, POWER);
        assertEquals(-1.0, result[0], EPSILON);
        assertEquals(0, result[1], EPSILON);
    }

    @Test
    void translationDiagonalPreservesDirection() {
        // A 45-degree input should produce a 45-degree output
        double[] result = SwerveCommands.applyTranslationScaling(0.5, 0.5, DEADBAND, POWER);

        // Both components should have the same sign and magnitude
        assertEquals(result[0], result[1], EPSILON);
        assertTrue(result[0] > 0);
    }

    @Test
    void translationDiagonalScaleMagnitudeIsLessThanLinear() {
        // With power=2, moderate inputs should be reduced (finer control near center)
        double[] result = SwerveCommands.applyTranslationScaling(0.5, 0, DEADBAND, POWER);
        assertTrue(result[0] > 0);
        assertTrue(result[0] < 0.5, "Power curve should reduce moderate inputs");
    }

    @Test
    void translationPower1GivesLinearResponse() {
        double[] result = SwerveCommands.applyTranslationScaling(0.55, 0, DEADBAND, 1.0);
        // With power=1 and deadband=0.1, input 0.55 should map to (0.55-0.1)/(1-0.1) = 0.5
        assertEquals(0.5, result[0], EPSILON);
    }

    // --- Rotation scaling ---

    @Test
    void rotationInsideDeadbandReturnsZero() {
        double result = SwerveCommands.applyRotationScaling(0.05, DEADBAND, POWER);
        assertEquals(0, result, EPSILON);
    }

    @Test
    void rotationZeroReturnsZero() {
        double result = SwerveCommands.applyRotationScaling(0, DEADBAND, POWER);
        assertEquals(0, result, EPSILON);
    }

    @Test
    void rotationFullPositiveReturnsOne() {
        double result = SwerveCommands.applyRotationScaling(1.0, DEADBAND, POWER);
        assertEquals(1.0, result, EPSILON);
    }

    @Test
    void rotationFullNegativeReturnsNegativeOne() {
        double result = SwerveCommands.applyRotationScaling(-1.0, DEADBAND, POWER);
        assertEquals(-1.0, result, EPSILON);
    }

    @Test
    void rotationPreservesSign() {
        double positive = SwerveCommands.applyRotationScaling(0.5, DEADBAND, POWER);
        double negative = SwerveCommands.applyRotationScaling(-0.5, DEADBAND, POWER);

        assertTrue(positive > 0);
        assertTrue(negative < 0);
        assertEquals(positive, -negative, EPSILON);
    }

    @Test
    void rotationPowerCurveReducesModerateInputs() {
        double result = SwerveCommands.applyRotationScaling(0.5, DEADBAND, POWER);
        assertTrue(result > 0);
        assertTrue(result < 0.5, "Power curve should reduce moderate inputs");
    }

    @Test
    void rotationPower1GivesLinearResponse() {
        double result = SwerveCommands.applyRotationScaling(0.55, DEADBAND, 1.0);
        // With power=1 and deadband=0.1, input 0.55 should map to (0.55-0.1)/(1-0.1) = 0.5
        assertEquals(0.5, result, EPSILON);
    }
}
