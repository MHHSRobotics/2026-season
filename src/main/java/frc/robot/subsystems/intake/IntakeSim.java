package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

public class IntakeSim extends SubsystemBase {
    private static final DCMotor rollerGearbox = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor hingeGearbox = DCMotor.getKrakenX60Foc(1);

    private MotorIO roller, hinge;
    private EncoderIO hingeEncoder;

    private DCMotorSim rollerMech;
    private SingleJointedArmSim hingeMech;

    public IntakeSim(MotorIO rollerIO, MotorIO hingeIO, EncoderIO hingeEncoderIO) {
        roller = rollerIO;
        hinge = hingeIO;
        hingeEncoder = hingeEncoderIO;
        rollerMech = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        rollerGearbox, Intake.Constants.rollerInertia, Intake.Constants.rollerRatio),
                rollerGearbox);
        hingeMech = new SingleJointedArmSim(
                hingeGearbox,
                Intake.Constants.hingeRatio,
                Intake.Constants.hingeInertia,
                1,
                Intake.Constants.hingeDown,
                Intake.Constants.hingeUp,
                false,
                Intake.Constants.hingeUp);
    }

    @Override
    public void periodic() {
        rollerMech.setInputVoltage(roller.getInputs().appliedVoltage);
        hingeMech.setInputVoltage(hinge.getInputs().appliedVoltage);

        rollerMech.update(Constants.loopTime);
        hingeMech.update(Constants.loopTime);

        roller.setMechPosition(rollerMech.getAngularPositionRad());
        roller.setMechVelocity(rollerMech.getAngularVelocityRadPerSec());

        hinge.setMechPosition(hingeMech.getAngleRads());
        hinge.setMechVelocity(hingeMech.getVelocityRadPerSec());

        hingeEncoder.setMechPosition(hingeMech.getAngleRads());
        hingeEncoder.setMechVelocity(hingeMech.getVelocityRadPerSec());
    }
}
