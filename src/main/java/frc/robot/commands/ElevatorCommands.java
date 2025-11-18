package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.elevator.Elevator;

// Class to get commands for the elevator. Separate from Elevator for organizational purposes.
public class ElevatorCommands {
    private Elevator elevator;

    public ElevatorCommands(Elevator elevator) {
        this.elevator = elevator;
    }

    // Tell the elevator motors how fast to spin (percent [-1 to 1], -1 = full down, 1 = full up)
    public Command setSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> elevator.setSpeed(speed.getAsDouble()), elevator)
                .withName("elevator set speed");
    }

    // Command to manually control the elevator at a fixed speed
    public Command setSpeed(double speed) {
        return setSpeed(() -> speed).withName("elevator set speed " + speed);
    }

    // Tell the elevator to stop all motor output
    public Command stop() {
        return setSpeed(0).withName("elevator stop");
    }

    // Tell the elevator to go to a target height (meters)
    public Command setGoal(DoubleSupplier goal) {
        return new InstantCommand(() -> elevator.setGoal(goal.getAsDouble()), elevator).withName("elevator set goal");
    }

    // Command to set the goal of the elevator to a fixed value (meters)
    public Command setGoal(double goal) {
        return setGoal(() -> goal).withName("elevator set goal " + goal);
    }

    // Tell the elevator to change its height by the given amount (meters, positive = up, negative = down)
    public Command changeGoal(DoubleSupplier change) {
        return new InstantCommand(() -> elevator.setGoal(elevator.getGoal() + change.getAsDouble()), elevator)
                .withName("elevator change goal");
    }

    // Command to increment the goal by a fixed amount (meters)
    public Command changeGoal(double increment) {
        return changeGoal(() -> increment).withName("elevator change goal " + increment);
    }
}
