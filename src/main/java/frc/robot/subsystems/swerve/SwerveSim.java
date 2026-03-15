package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSim extends SubsystemBase {
    protected Pose3d currentPose;

    public Pose3d getPhysicalPose() {
        return currentPose;
    }
}
