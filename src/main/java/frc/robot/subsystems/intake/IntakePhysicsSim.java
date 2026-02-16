package frc.robot.subsystems.intake;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.io.MotorIO;

public class IntakePhysicsSim extends SubsystemBase {
    private MotorIO hinge;
    private MotorIO flywheel;

    private DoubleSubscriber hingePos;
    private DoubleSubscriber hingeVel;
    private DoubleSubscriber flywheelPos;
    private DoubleSubscriber flywheelVel;

    public IntakePhysicsSim(MotorIO flywheelIO, MotorIO hingeIO, String path) {
        hinge = hingeIO;
        flywheel = flywheelIO;
        hingePos = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Hinge/Position")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
        hingeVel = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Hinge/Velocity")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
        flywheelPos = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Flywheel/Position")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
        flywheelVel = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Flywheel/Velocity")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
    }

    @Override
    public void periodic() {
        hinge.setMechPosition(hingePos.get());
        hinge.setMechVelocity(hingeVel.get());
        flywheel.setMechPosition(flywheelPos.get());
        flywheel.setMechVelocity(flywheelVel.get());
    }
}
