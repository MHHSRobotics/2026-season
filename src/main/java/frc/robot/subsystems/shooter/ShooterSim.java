package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.MotorIO;

public class ShooterSim extends SubsystemBase{
    private static final DCMotor feedGearbox=DCMotor.getKrakenX60Foc(1);
    private static final DCMotor flyGearbox=DCMotor.getKrakenX60Foc(1);

    private MotorIO feed,fly;

    private DCMotorSim feedMech;
    private DCMotorSim flyMech;

    public ShooterSim(MotorIO feedIO, MotorIO flyIO){
        feed=feedIO;
        fly=flyIO;
        feedMech=new DCMotorSim(LinearSystemId.createDCMotorSystem(feedGearbox,Shooter.Constants.feedInertia,Shooter.Constants.feedRatio), feedGearbox);
        flyMech=new DCMotorSim(LinearSystemId.createDCMotorSystem(flyGearbox,Shooter.Constants.flyInertia,Shooter.Constants.flyRatio), flyGearbox);
    }

    @Override
    public void periodic(){
        feedMech.setInputVoltage(feed.getInputs().appliedVoltage);
        flyMech.setInputVoltage(fly.getInputs().appliedVoltage);

        feedMech.update(Constants.loopTime);
        flyMech.update(Constants.loopTime);

        feed.setMechPosition(feedMech.getAngularPositionRad());
        feed.setMechVelocity(feedMech.getAngularVelocityRadPerSec());

        fly.setMechPosition(flyMech.getAngularPositionRad());
        fly.setMechVelocity(flyMech.getAngularVelocityRadPerSec());
    }
}
