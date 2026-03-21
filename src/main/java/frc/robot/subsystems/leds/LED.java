package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.signals.RGBWColor;

import org.littletonrobotics.junction.Logger;

import frc.robot.io.LedIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class LED extends SubsystemBase {
    public static class Constants {
        public static final int startIndex = 8;
        public static final int endIndex = 100;

        public static final int id = 22;
    }

    private LedIO leds;
    private Shooter shooter;
    private Swerve swerve;
    private RGBWColor currentColor;

    public LED(LedIO ledIO, Shooter shooter, Swerve swerve) {
        leds = ledIO;
        this.shooter = shooter;
        this.swerve = swerve;
        currentColor = new RGBWColor(0, 0, 0);
    }

    public void setColor(RGBWColor color) {
        if (currentColor.Red != color.Red || currentColor.Green != color.Green || currentColor.Blue != color.Blue) {
            leds.setColor(Constants.startIndex, Constants.endIndex, color);
            currentColor = color;
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput("LED/Color", new int[] {currentColor.Red, currentColor.Green, currentColor.Blue});
        if (shooter.getFlyVelocity() > -10000000) {
            if (shooter.atTargetSpeed() && shooter.getFlyVelocity() > 10) {
                setColor(new RGBWColor(255, 0, 0));
            } else {
                setColor(new RGBWColor(0, 255, 0));
            }
        } else {
            setColor(new RGBWColor());
        }
    }
}
