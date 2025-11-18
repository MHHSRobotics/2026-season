package frc.robot;

import edu.wpi.first.math.util.Units;

// RobotPositions has arm, elevator, and wrist positions for each of the standard positions (default, L1, L2, L3, L4,
// low algae, high algae, source)
public class RobotPositions {
    public static final double armDefault = Units.degreesToRadians(90);
    public static final double elevatorDefault = 0.05;
    public static final double wristDefault = Units.degreesToRadians(90);

    public static final double armL1 = 2.06;
    public static final double elevatorL1 = 0.1;
    public static final double wristL1 = 0.22;

    public static final double armL2 = 2.25;
    public static final double elevatorL2 = 0.3;
    public static final double wristL2 = 0;

    public static final double armL3 = 2.25;
    public static final double elevatorL3 = 0.76;
    public static final double wristL3 = 0.23;

    public static final double armL4 = 2.3;
    public static final double elevatorL4 = 1.12;
    public static final double wristL4 = -0.44;

    public static final double armLowAlgae = Units.degreesToRadians(0);
    public static final double elevatorLowAlgae = 0;
    public static final double wristLowAlgae = Units.degreesToRadians(0);

    public static final double armHighAlgae = Units.degreesToRadians(0);
    public static final double elevatorHighAlgae = 0;
    public static final double wristHighAlgae = Units.degreesToRadians(0);

    public static final double armSource = Units.degreesToRadians(40);
    public static final double elevatorSource = 0.4;
    public static final double wristSource = Units.degreesToRadians(-90);
}
