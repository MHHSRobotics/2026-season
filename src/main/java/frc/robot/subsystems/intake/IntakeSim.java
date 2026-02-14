package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.io.MotorIO;

public class IntakeSim extends SubsystemBase {
    private static final DCMotor intakeGearbox = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor hingeGearbox = DCMotor.getKrakenX60Foc(1);

    private MotorIO intake, hinge;

    private DCMotorSim intakeMech;
    private SingleJointedArmSim hingeMech;

    public IntakeSim(MotorIO intakeIO, MotorIO hingeIO) {
        intake = intakeIO;
        hinge = hingeIO;
        intakeMech = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        intakeGearbox, Intake.Constants.intakeInertia, Intake.Constants.intakeRatio),
                intakeGearbox);
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
        intakeMech.setInputVoltage(intake.getInputs().appliedVoltage);
        hingeMech.setInputVoltage(hinge.getInputs().appliedVoltage);

        intakeMech.update(Constants.loopTime);
        hingeMech.update(Constants.loopTime);

        intake.setMechPosition(intakeMech.getAngularPositionRad());
        intake.setMechVelocity(intakeMech.getAngularVelocityRadPerSec());

        hinge.setMechPosition(hingeMech.getAngleRads());
        hinge.setMechVelocity(hingeMech.getVelocityRadPerSec());
    }
}
