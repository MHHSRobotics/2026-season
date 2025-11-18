package frc.robot.subsystems.wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

// Make the wrist simulator act like the real robot wrist so students can test code without hardware.
// The simulator uses the same constants as the real wrist and updates angle (radians) and speed (rad/s).
public class WristSim extends SubsystemBase {
    // Interfaces for the motor and encoder (these talk to the sim instead of real hardware)
    private MotorIO motor;
    private EncoderIO encoder;

    // Motor model: one Kraken X60 FOC motor on the wrist gearbox
    private static final DCMotor wristGearbox = DCMotor.getKrakenX60Foc(1);
    // Simulation model for a single-jointed wrist:
    // - Input: voltage applied by the motor (volts)
    // - Outputs: wrist angle (radians) and wrist angular velocity (radians per second)
    private SingleJointedArmSim wristMech;

    // Make the wrist simulator with the same ratios, limits, and starting angle (radians) as the real wrist
    public WristSim(MotorIO motor, EncoderIO encoder) {
        this.motor = motor;
        this.encoder = encoder;
        wristMech = new SingleJointedArmSim(
                wristGearbox,
                Wrist.Constants.gearRatio,
                Wrist.Constants.momentOfInertia,
                Wrist.Constants.wristLength,
                Wrist.Constants.minAngle,
                Wrist.Constants.maxAngle,
                false,
                Wrist.Constants.startAngle);
    }

    @Override
    public void periodic() {
        // This runs every robot loop (about 50 times per second)
        // 1) Tell the simulator what voltage (volts) the motor applied
        wristMech.setInputVoltage(motor.getInputs().appliedVoltage);

        // 2) Step the simulator forward by 20 ms (0.02 seconds)
        wristMech.update(0.02);

        // 3) Tell the motor and encoder I/O the new wrist angle and speed
        // All values here are mechanism radians (rad) and radians per second (rad/s)
        motor.setMechPosition(wristMech.getAngleRads());
        motor.setMechVelocity(wristMech.getVelocityRadPerSec());
        encoder.setMechPosition(wristMech.getAngleRads());
        encoder.setMechVelocity(wristMech.getVelocityRadPerSec());
    }
}
