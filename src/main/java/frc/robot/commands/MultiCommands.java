package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.signals.RGBWColor;

public class MultiCommands {
    private HopperCommands hopper;
    private IntakeCommands intake;
    private ShooterCommands shooter;
    private LEDCommands led;

    public MultiCommands(HopperCommands hopper, IntakeCommands intake, ShooterCommands shooter, LEDCommands led) {
        this.hopper = hopper;
        this.intake = intake;
        this.shooter = shooter;
        this.led = led;
    }

    public Command shoot() {
        return hopper.forward()
                .alongWith(
                        shooter.feedShootWhenAtTarget(),
                        shooter.flyShoot(),
                        led.startEndColor(new RGBWColor(0, 255, 0), new RGBWColor(0, 0, 0)));
    }
}
