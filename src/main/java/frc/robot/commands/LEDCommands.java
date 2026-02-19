package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.subsystems.leds.LED;

public class LEDCommands {
    private LED led;

    public LEDCommands(LED led) {
        this.led = led;
    }

    public Command setColor(RGBWColor color) {
        return new InstantCommand(() -> led.setColor(color));
    }

    public Command startEndColor(RGBWColor startColor, RGBWColor endColor) {
        return Commands.startEnd(() -> led.setColor(startColor), () -> led.setColor(endColor), led);
    }
}
