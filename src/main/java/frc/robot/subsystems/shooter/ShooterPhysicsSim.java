package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.io.MotorIO;

public class ShooterPhysicsSim extends SubsystemBase {
    private MotorIO feed, fly, fly2;

    private DoubleSubscriber feedPos;
    private DoubleSubscriber feedVel;
    private DoubleSubscriber flywheelPos;
    private DoubleSubscriber flywheelVel;

    public ShooterPhysicsSim(MotorIO feedIO, MotorIO flywheelIO, MotorIO flywheelIO2, String path) {
        feed = feedIO;
        fly = flywheelIO;
        fly2 = flywheelIO2;
        feedPos = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Feed/Position")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
        feedVel = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Feed/Velocity")
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
        feed.setMechPosition(feedPos.get());
        feed.setMechVelocity(feedVel.get());
        fly.setMechPosition(flywheelPos.get());
        fly.setMechVelocity(flywheelVel.get());
        fly2.setMechPosition(flywheelPos.get());
        fly2.setMechVelocity(flywheelVel.get());
    }
}
