package frc.robot.network;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.wrist.Wrist;

// Class that publishes 3D robot data to AdvantageScope
public class RobotPublisher {
    private Arm arm;
    private Wrist wrist;
    private Intake intake;
    private Elevator elevator;
    private Hang hang;
    private Swerve swerve;

    public RobotPublisher(Arm arm, Wrist wrist, Intake intake, Elevator elevator, Hang hang, Swerve swerve) {
        this.arm = arm;
        this.wrist = wrist;
        this.intake = intake;
        this.elevator = elevator;
        this.hang = hang;
        this.swerve = swerve;
    }

    public void publish() {
        Pose2d pos = swerve.getPose();
        // Rotate 180 degrees because the orientation of the robot model is wrong
        pos = pos.transformBy(new Transform2d(new Translation2d(), Rotation2d.k180deg));
        Pose3d botPos = new Pose3d(pos);

        // Position of the main link needs to be absolute, so add botPos to chassisPos for main link
        Transform3d chassisPos = new Transform3d(0, 0, 0.04, new Rotation3d());
        Logger.recordOutput("3DField/Chassis", botPos.plus(chassisPos));

        // Convert chassisPos to Pose3d, other links need relative positions
        Pose3d mainPos = new Pose3d(chassisPos.getTranslation(), chassisPos.getRotation());

        Pose3d elevatorMiddle =
                mainPos.plus(new Transform3d(0, 0.0635, elevator.getPosition() / 2 + 0.1, new Rotation3d()));
        Logger.recordOutput("3DField/ElevatorMiddle", elevatorMiddle);

        Pose3d elevatorInner =
                elevatorMiddle.plus(new Transform3d(0, 0, elevator.getPosition() / 2 + 0.03, new Rotation3d()));
        Logger.recordOutput("3DField/ElevatorInner", elevatorInner);

        // Subtract pi/2 because the arm was vertical in Fusion
        Pose3d armPos = elevatorInner.plus(
                new Transform3d(-0.1143, 0.1284, 0.108, new Rotation3d(0, arm.getPosition() - Math.PI / 2, 0)));
        Logger.recordOutput("3DField/Arm", armPos);

        Pose3d wristPos = armPos.plus(new Transform3d(0, -0.004, 0.5944, new Rotation3d(0, wrist.getPosition(), 0)));
        Logger.recordOutput("3DField/Wrist", wristPos);
    }
}
