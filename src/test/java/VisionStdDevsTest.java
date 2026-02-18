import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.VisionConstants;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/** Tests for Swerve.calculateVisionStdDevs() vision trust weighting. */
public class VisionStdDevsTest {

    private static final double EPSILON = 1e-9;

    // Helper to extract values from the 3x1 matrix
    private double xyStdDev(Matrix<N3, N1> m) {
        return m.get(0, 0);
    }

    private double thetaStdDev(Matrix<N3, N1> m) {
        return m.get(2, 0);
    }

    // --- Single tag vs multi tag ---

    @Test
    void multiTagGivesTighterStdDevsThanSingleTag() {
        Matrix<N3, N1> single = Swerve.calculateVisionStdDevs(2.0, 0, 1);
        Matrix<N3, N1> multi = Swerve.calculateVisionStdDevs(2.0, 0, 2);

        assertTrue(xyStdDev(multi) < xyStdDev(single));
        assertTrue(thetaStdDev(multi) < thetaStdDev(single));
    }

    // --- Distance scaling ---

    @Test
    void closerDistanceGivesTighterStdDevs() {
        Matrix<N3, N1> close = Swerve.calculateVisionStdDevs(1.0, 0, 1);
        Matrix<N3, N1> far = Swerve.calculateVisionStdDevs(5.0, 0, 1);

        assertTrue(xyStdDev(close) < xyStdDev(far));
        assertTrue(thetaStdDev(close) < thetaStdDev(far));
    }

    @Test
    void zeroDistanceSingleTagReturnsBaseStdDevs() {
        Matrix<N3, N1> result = Swerve.calculateVisionStdDevs(0, 0, 1);

        assertEquals(VisionConstants.singleTagXYStdDev, xyStdDev(result), EPSILON);
        assertEquals(VisionConstants.singleTagThetaStdDev, thetaStdDev(result), EPSILON);
    }

    @Test
    void zeroDistanceMultiTagReturnsBaseStdDevs() {
        Matrix<N3, N1> result = Swerve.calculateVisionStdDevs(0, 0, 2);

        assertEquals(VisionConstants.multiTagXYStdDev, xyStdDev(result), EPSILON);
        assertEquals(VisionConstants.multiTagThetaStdDev, thetaStdDev(result), EPSILON);
    }

    @Test
    void xyScalesWithDistanceSquared() {
        // At distance d, xyStdDev = base + d^2 * multiplier
        double distance = 3.0;
        Matrix<N3, N1> result = Swerve.calculateVisionStdDevs(distance, 0, 1);
        double expected = VisionConstants.singleTagXYStdDev
                + (distance * distance * VisionConstants.visionXYStdDevDistanceMultiplier);

        assertEquals(expected, xyStdDev(result), EPSILON);
    }

    @Test
    void thetaScalesWithDistance() {
        // At distance d, thetaStdDev = base + d * multiplier
        double distance = 3.0;
        Matrix<N3, N1> result = Swerve.calculateVisionStdDevs(distance, 0, 1);
        double expected =
                VisionConstants.singleTagThetaStdDev + (distance * VisionConstants.visionThetaStdDevDistanceMultiplier);

        assertEquals(expected, thetaStdDev(result), EPSILON);
    }

    // --- Ambiguity scaling ---

    @Test
    void higherAmbiguityGivesLooserStdDevs() {
        Matrix<N3, N1> lowAmb = Swerve.calculateVisionStdDevs(2.0, 0.1, 1);
        Matrix<N3, N1> highAmb = Swerve.calculateVisionStdDevs(2.0, 0.5, 1);

        assertTrue(xyStdDev(highAmb) > xyStdDev(lowAmb));
        assertTrue(thetaStdDev(highAmb) > thetaStdDev(lowAmb));
    }

    @Test
    void zeroAmbiguityDoesNotScale() {
        // ambiguity=0 means multiply by (1+0)=1, so no change from base+distance
        double distance = 2.0;
        Matrix<N3, N1> result = Swerve.calculateVisionStdDevs(distance, 0, 1);

        double expectedXY = VisionConstants.singleTagXYStdDev
                + (distance * distance * VisionConstants.visionXYStdDevDistanceMultiplier);
        double expectedTheta =
                VisionConstants.singleTagThetaStdDev + (distance * VisionConstants.visionThetaStdDevDistanceMultiplier);

        assertEquals(expectedXY, xyStdDev(result), EPSILON);
        assertEquals(expectedTheta, thetaStdDev(result), EPSILON);
    }

    @Test
    void ambiguityScalesMultiplicatively() {
        double distance = 2.0;
        double ambiguity = 0.3;
        Matrix<N3, N1> result = Swerve.calculateVisionStdDevs(distance, ambiguity, 1);

        double baseXY = VisionConstants.singleTagXYStdDev
                + (distance * distance * VisionConstants.visionXYStdDevDistanceMultiplier);
        double expectedXY = baseXY * (1 + ambiguity);

        assertEquals(expectedXY, xyStdDev(result), EPSILON);
    }

    // --- Both XY components are equal ---

    @Test
    void xyStdDevsAreEqual() {
        Matrix<N3, N1> result = Swerve.calculateVisionStdDevs(3.0, 0.2, 1);
        assertEquals(result.get(0, 0), result.get(1, 0), EPSILON);
    }
}
