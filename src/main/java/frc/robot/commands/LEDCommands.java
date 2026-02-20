package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.subsystems.leds.LED;

public class LEDCommands {
    private LED led;

    public LEDCommands(LED led) {
        this.led = led;
    }

    public Command setColor(Supplier<RGBWColor> color) {
        return Commands.runEnd(() -> led.setColor(color.get()), () -> led.setColor(new RGBWColor()));
    }

    public Command startEndColor(RGBWColor startColor, RGBWColor endColor) {
        return Commands.startEnd(() -> led.setColor(startColor), () -> led.setColor(endColor), led);
    }
}
