package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.wrist.Wrist;

// Class to get commands for the wrist. Separate from Wrist for organizational purposes.
public class WristCommands {
    private Wrist wrist;

    public WristCommands(Wrist wrist) {
        this.wrist = wrist;
    }

    // Tell the wrist motor how fast to spin (percent [-1 to 1], -1 = full backward, 1 = full forward)
    public Command setSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> wrist.setSpeed(speed.getAsDouble()), wrist).withName("wrist set duty cycle");
    }

    // Command to manually control the wrist at a fixed speed
    public Command setSpeed(double speed) {
        return setSpeed(() -> speed).withName("wrist set speed " + speed);
    }

    // Tell the wrist to stop all motor output
    public Command stop() {
        return setSpeed(0).withName("wrist stop");
    }

    // Tell the wrist to go to a target angle (radians, like 0 = straight forward)
    public Command setGoal(DoubleSupplier goal) {
        return new InstantCommand(() -> wrist.setGoal(goal.getAsDouble()), wrist).withName("wrist set goal");
    }

    // Command to set the goal of the wrist to a fixed value (radians)
    public Command setGoal(double goal) {
        return setGoal(() -> goal).withName("wrist set goal " + goal);
    }

    // Tell the wrist to change its angle by the given amount (radians, positive = up, negative = down)
    public Command changeGoal(DoubleSupplier change) {
        return new InstantCommand(() -> wrist.setGoal(wrist.getGoal() + change.getAsDouble()), wrist)
                .withName("wrist change goal");
    }

    // Command to increment the goal by a fixed amount (radians)
    public Command changeGoal(double increment) {
        return changeGoal(() -> increment).withName("wrist change goal " + increment);
    }
}
