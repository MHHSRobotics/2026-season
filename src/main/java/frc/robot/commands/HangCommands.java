package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.hang.Hang;

// Class to get commands for the hang mechanism. Simple speed control for climbing.
public class HangCommands {
    private Hang hang;

    public HangCommands(Hang hang) {
        this.hang = hang;
    }

    // Tell the hang motor how fast to spin (percent [-1 to 1], -1 = full down, 1 = full up)
    public Command setSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> hang.setSpeed(speed.getAsDouble()), hang).withName("hang set speed");
    }

    // Command to manually control the hang at a fixed speed
    public Command setSpeed(double speed) {
        return setSpeed(() -> speed).withName("hang set speed " + speed);
    }

    // Tell the hang motor to stop all movement
    public Command stop() {
        return setSpeed(0.0).withName("hang stop");
    }

    // Tell the hang to extend up at full speed (for climbing up)
    public Command extendUp() {
        return setSpeed(1.0).withName("hang extend up");
    }

    // Tell the hang to retract down at full speed (for lowering down)
    public Command retractDown() {
        return setSpeed(-1.0).withName("hang retract down");
    }
}
