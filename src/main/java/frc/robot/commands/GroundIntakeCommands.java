package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.GroundIntake;

public class GroundIntakeCommands {

    private GroundIntake groundIntake;

    public GroundIntakeCommands(GroundIntake gIntake) {
        this.groundIntake = gIntake;
    }

    // Stops all swerve output
    public Command stop() {
        return new InstantCommand(() -> groundIntake.stop());
    }
/* 
    public Command setForward(double radPerSecond) {
        return new InstantCommand(() -> groundIntake.setForward(radPerSecond));
    }
*/
    public Command setDown(double radPerSecond) {
        return new InstantCommand(() -> groundIntake.setDown(radPerSecond));
    }

    public Command setLocked(boolean brake) {
        return new InstantCommand(() -> groundIntake.setLocked(brake));
    }
}
