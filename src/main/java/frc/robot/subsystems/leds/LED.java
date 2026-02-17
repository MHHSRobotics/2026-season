package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.io.LedIO;

public class LED extends SubsystemBase {
    public static class Constants {
        public static final int startIndex = 0;
        public static final int endIndex = 59;

        public static final int id = 20;
    }

    private LedIO leds;

    public LED(LedIO ledIO) {
        leds = ledIO;
    }

    public void setColor(RGBWColor color) {
        leds.setColor(Constants.startIndex, Constants.endIndex, color);
    }

    @Override
    public void periodic() {
        int m = (int) (Math.random() * 255);
        leds.setColor(0, (int) (Math.random() * Constants.endIndex), new RGBWColor(0, 0, 0, m));
    }
}
