package frc.robot.subsystems.intake;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

public class IntakePhysicsSim extends SubsystemBase {
    private MotorIO hinge;
    private MotorIO roller;
    private EncoderIO hingeEncoder;

    private DoubleSubscriber hingePos;
    private DoubleSubscriber hingeVel;
    private DoubleSubscriber rollerPos;
    private DoubleSubscriber rollerVel;

    public IntakePhysicsSim(MotorIO rollerIO, MotorIO hingeIO, EncoderIO hingeEncoderIO, String path) {
        hinge = hingeIO;
        roller = rollerIO;
        hingeEncoder = hingeEncoderIO;
        hingePos = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Hinge/Position")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
        hingeVel = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Hinge/Velocity")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
        rollerPos = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Roller/Position")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
        rollerVel = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Roller/Velocity")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
    }

    @Override
    public void periodic() {
        hinge.setMechPosition(hingePos.get());
        hinge.setMechVelocity(hingeVel.get());
        hingeEncoder.setMechPosition(hingePos.get());
        hingeEncoder.setMechVelocity(hingeVel.get());
        roller.setMechPosition(rollerPos.get());
        roller.setMechVelocity(rollerVel.get());
    }
}
