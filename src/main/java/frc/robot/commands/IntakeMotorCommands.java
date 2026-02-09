package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeMotor;

public class IntakeMotorCommands extends InstantCommand {
    public IntakeMotorCommands(IntakeMotor intake) {
        super(intake::intakeOff, intake);
        setName("intake off");
    }
}
