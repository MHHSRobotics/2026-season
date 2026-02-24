package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.signals.RGBWColor;

import org.littletonrobotics.junction.Logger;

import frc.robot.io.LedIO;

public class LED extends SubsystemBase {
    public static class Constants {
        public static final int startIndex = 0;
        public static final int endIndex = 59;

        public static final int id = 20;
    }

    private LedIO leds;
    private RGBWColor currentColor;

    public LED(LedIO ledIO) {
        leds = ledIO;
        currentColor = new RGBWColor(0, 0, 0);
    }

    public void setColor(RGBWColor color) {
        leds.setColor(Constants.startIndex, Constants.endIndex, color);
        currentColor = color;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("LED/Color", new int[] {currentColor.Red, currentColor.Green, currentColor.Blue});
    }
}
