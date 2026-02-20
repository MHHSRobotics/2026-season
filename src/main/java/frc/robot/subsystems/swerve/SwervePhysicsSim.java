package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructSubscriber;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;

public class SwervePhysicsSim extends SwerveSim {
    private StructSubscriber<Pose3d> sub;

    public SwervePhysicsSim(String path) {
        sub = NetworkTableInstance.getDefault()
                .getStructTopic(path, Pose3d.struct)
                .subscribe(new Pose3d(), PubSubOption.periodic(Constants.loopTime));
    }

    @Override
    public void periodic() {
        currentPose = sub.get();
        Logger.recordOutput("SwerveSim/ActualPose", currentPose);
    }
}
