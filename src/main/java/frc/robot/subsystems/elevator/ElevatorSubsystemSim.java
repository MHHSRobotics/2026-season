package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

// Make the elevator simulator act like the real robot elevator so students can test code without hardware.
// The simulator uses the same constants as the real elevator and updates height (meters) and speed (m/s).
public class ElevatorSubsystemSim extends SubsystemBase {
    // Interfaces for the motors and encoder (these talk to the sim instead of real hardware)
    private MotorIO leftMotor;
    private MotorIO rightMotor;
    private EncoderIO encoder;

    // Motor model: two Falcon 500 FOC motors on the elevator gearbox
    private static final DCMotor elevatorGearbox = DCMotor.getFalcon500Foc(2);
    // Simulation model for an elevator:
    // - Input: voltage applied by the motors (volts)
    // - Outputs: elevator height (meters) and elevator velocity (meters per second)
    private ElevatorSim elevatorMech;

    // Make the elevator simulator with the same ratios, limits, and starting height (meters) as the real elevator
    public ElevatorSubsystemSim(MotorIO leftMotor, MotorIO rightMotor, EncoderIO encoder) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.encoder = encoder;
        elevatorMech = new ElevatorSim(
                elevatorGearbox,
                Elevator.Constants.gearRatio,
                Elevator.Constants.carriageMass,
                Elevator.Constants.drumRadius,
                Elevator.Constants.minHeight,
                Elevator.Constants.maxHeight,
                true,
                Elevator.Constants.startHeight);
    }

    @Override
    public void periodic() {
        // This runs every robot loop (about 50 times per second)
        // 1) Tell the simulator what voltage (volts) the left motor applied
        // (Right motor follows left motor, so we only need to simulate the left motor's voltage)
        elevatorMech.setInputVoltage(leftMotor.getInputs().appliedVoltage);

        // 2) Step the simulator forward by 20 ms (0.02 seconds)
        elevatorMech.update(0.02);

        // 3) Tell the motor and encoder I/O the new elevator height and speed
        // All values here are mechanism meters (m) and meters per second (m/s)
        leftMotor.setMechPosition(elevatorMech.getPositionMeters());
        leftMotor.setMechVelocity(elevatorMech.getVelocityMetersPerSecond());

        // Right motor gets the same position and velocity since it follows the left motor
        rightMotor.setMechPosition(elevatorMech.getPositionMeters());
        rightMotor.setMechVelocity(elevatorMech.getVelocityMetersPerSecond());

        // Encoder also gets the same values
        encoder.setMechPosition(elevatorMech.getPositionMeters());
        encoder.setMechVelocity(elevatorMech.getVelocityMetersPerSecond());
    }
}
