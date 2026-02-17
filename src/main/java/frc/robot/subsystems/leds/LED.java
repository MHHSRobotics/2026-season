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
    private int t;

    public LED(LedIO ledIO) {
        leds = ledIO;
    }

    public void setColor(RGBWColor color) {
        leds.setColor(Constants.startIndex, Constants.endIndex, color);
    }

    @Override
    public void periodic() {
        t += 1;
        int r = (int) (Math.random() * 255);
        int g = (int) (Math.random() * 255);
        int b = (int) (Math.random() * 255);
        leds.setColor(t, t, new RGBWColor(r, g, b));
        if (t >= 30) {
            t = 10;
        }
    }
}
