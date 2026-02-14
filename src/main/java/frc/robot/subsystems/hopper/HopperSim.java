package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.MotorIO;

public class HopperSim extends SubsystemBase{
    private static final DCMotor rollerGearbox=DCMotor.getKrakenX60Foc(1);

    private MotorIO roller;

    private DCMotorSim rollerMech;

    public HopperSim(MotorIO rollerIO){
        roller=rollerIO;
        rollerMech=new DCMotorSim(LinearSystemId.createDCMotorSystem(rollerGearbox,Hopper.Constants.rollerInertia,Hopper.Constants.rollerRatio), rollerGearbox);
    }

    @Override
    public void periodic(){
        rollerMech.setInputVoltage(roller.getInputs().appliedVoltage);

        rollerMech.update(Constants.loopTime);

        roller.setMechPosition(rollerMech.getAngularPositionRad());
        roller.setMechVelocity(rollerMech.getAngularVelocityRadPerSec());
    }
}
