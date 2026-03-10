package frc.robot.subsystems.swerve;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

public class SwerveModulePhysicsSim extends SubsystemBase {
    private MotorIO driveMotor;
    private MotorIO steerMotor;
    private EncoderIO steerEncoder;

    private DoubleSubscriber drivePos;
    private DoubleSubscriber driveVel;
    private DoubleSubscriber steerPos;
    private DoubleSubscriber steerVel;

    public SwerveModulePhysicsSim(MotorIO driveMotorIO, MotorIO steerMotorIO, EncoderIO steerEncoderIO, String path) {
        driveMotor = driveMotorIO;
        steerMotor = steerMotorIO;
        steerEncoder = steerEncoderIO;

        drivePos = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Drive/Position")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
        driveVel = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Drive/Velocity")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
        steerPos = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Steer/Angle")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
        steerVel = NetworkTableInstance.getDefault()
                .getDoubleTopic(path + "/Steer/Velocity")
                .subscribe(0, PubSubOption.periodic(Constants.loopTime));
    }

    @Override
    public void periodic() {
        driveMotor.setMechPosition(drivePos.get());
        driveMotor.setMechVelocity(driveVel.get());

        steerMotor.setMechPosition(steerPos.get());
        steerMotor.setMechVelocity(steerVel.get());

        steerEncoder.setMechPosition(steerPos.get());
        steerEncoder.setMechVelocity(steerVel.get());
    }
}
