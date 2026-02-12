package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.intake.GroundIntake;

public class GroundIntakeCommands {

    private GroundIntake groundIntake;

    public GroundIntakeCommands(GroundIntake gIntake) {
        this.groundIntake = gIntake;
    }

    // Stops all swerve output
    public Command stop() {
        return new InstantCommand(() -> groundIntake.stop());
    }

    public Command setPos(double speed) {
        return new InstantCommand(() -> groundIntake.setPos(speed));
    }

    public Command setDown(double speed) {
        return new InstantCommand(() -> groundIntake.setDown(speed));
    }

    public Command setUp(double speed) {
        return new InstantCommand(() -> groundIntake.setForward(speed));
    }

    public Command setSpeed(double speed) {
        return new InstantCommand(() -> groundIntake.setSpeed(speed));
    }

    public Command setLocked(boolean brake) {
        return new InstantCommand(() -> groundIntake.setLocked(brake));
    }

    public Command intakeForward() {
        return new InstantCommand(() -> groundIntake.intakeOn());
    }

    public Command intakeReverse() {
        return new InstantCommand(() -> groundIntake.intakeReverse());
    }

    public Command intakeStop() {
        return new InstantCommand(() -> groundIntake.intakeOff());
    }
}
