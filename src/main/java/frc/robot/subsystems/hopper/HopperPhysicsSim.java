package frc.robot.subsystems.hopper;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.io.MotorIO;

public class HopperPhysicsSim extends SubsystemBase {
    private MotorIO roller;

    private DoubleSubscriber rollerPos;
    private DoubleSubscriber rollerVel;

    public HopperPhysicsSim(MotorIO rollerIO, String path) {
        roller = rollerIO;
        rollerPos = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Roller/Position")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
        rollerVel = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Roller/Velocity")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
    }

    @Override
    public void periodic() {
        roller.setMechPosition(rollerPos.get());
        roller.setMechVelocity(rollerVel.get());
    }
}
