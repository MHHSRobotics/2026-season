package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class MultiCommands {
    private HopperCommands hopperCommands;
    private IntakeCommands intakeCommands;
    private ShooterCommands shooterCommands;
    private LEDCommands ledCommands;
    private Shooter shooter;

    public MultiCommands(
            HopperCommands hopperCommands,
            IntakeCommands intakeCommands,
            ShooterCommands shooterCommands,
            LEDCommands ledCommands,
            Shooter shooter) {
        this.hopperCommands = hopperCommands;
        this.intakeCommands = intakeCommands;
        this.shooterCommands = shooterCommands;
        this.ledCommands = ledCommands;
        this.shooter = shooter;
    }

    public Command shoot() {
        if(ledCommands!=null){
            return hopperCommands
                .setSpeed(() -> shooter.atTargetSpeed() ? Hopper.Constants.rollerSpeed : 0)
                .alongWith(
                        shooterCommands.setFeedSpeed(() -> shooter.atTargetSpeed() ? Shooter.Constants.feedSpeed : 0),
                        shooterCommands.flyShoot(),
                        ledCommands.setColor(
                                () -> shooter.atTargetSpeed() ? new RGBWColor(0, 255, 0) : new RGBWColor(255, 0, 0)));
        }else{
            return hopperCommands
                .setSpeed(() -> shooter.atTargetSpeed() ? Hopper.Constants.rollerSpeed : 0)
                .alongWith(
                        shooterCommands.setFeedSpeed(() -> shooter.atTargetSpeed() ? Shooter.Constants.feedSpeed : 0),
                        shooterCommands.flyShoot());
        }
        
    }
}
