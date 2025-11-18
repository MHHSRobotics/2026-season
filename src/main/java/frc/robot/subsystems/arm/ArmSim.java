package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

// Make the arm simulator act like the real robot arm so students can test code without hardware.
// The simulator uses the same constants as the real arm and updates angle (radians) and speed (rad/s).
public class ArmSim extends SubsystemBase {
    // Interfaces for the motor and encoder (these talk to the sim instead of real hardware)
    private MotorIO motor;
    private EncoderIO encoder;

    // Motor model: one Kraken X60 FOC motor on the arm gearbox
    private static final DCMotor armGearbox = DCMotor.getKrakenX60Foc(1);
    // Simulation model for a single-jointed arm:
    // - Input: voltage applied by the motor (volts)
    // - Outputs: arm angle (radians) and arm angular velocity (radians per second)
    private SingleJointedArmSim armMech;

    // Make the arm simulator with the same ratios, limits, and starting angle (radians) as the real arm
    public ArmSim(MotorIO motor, EncoderIO encoder) {
        this.motor = motor;
        this.encoder = encoder;
        armMech = new SingleJointedArmSim(
                armGearbox,
                Arm.Constants.gearRatio,
                Arm.Constants.moi,
                Arm.Constants.armLength,
                Arm.Constants.minAngle,
                Arm.Constants.maxAngle,
                true,
                Arm.Constants.startAngle);
    }

    @Override
    public void periodic() {
        // This runs every robot loop (about 50 times per second)
        // 1) Tell the simulator what voltage (volts) the motor applied
        armMech.setInputVoltage(motor.getInputs().appliedVoltage);

        // 2) Step the simulator forward by 20 ms (0.02 seconds)
        armMech.update(0.02);

        // 3) Tell the motor and encoder I/O the new arm angle and speed
        // All values here are mechanism radians (rad) and radians per second (rad/s)
        motor.setMechPosition(armMech.getAngleRads());
        motor.setMechVelocity(armMech.getVelocityRadPerSec());
        encoder.setMechPosition(armMech.getAngleRads());
        encoder.setMechVelocity(armMech.getVelocityRadPerSec());
    }
}
