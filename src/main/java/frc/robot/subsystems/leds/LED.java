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
        public static final int endIndex = 20;

        public static final int backId = 22;
        public static final int frontId = 21;
    }

    private LedIO frontCandle;
    private LedIO backCandle;
    private Shooter shooter;
    private Swerve swerve;
    private RGBWColor currentColor;

    public LED(LedIO frontCandle, LedIO backCandle, Shooter shooter, Swerve swerve) {
        this.frontCandle = frontCandle;
        this.backCandle = backCandle;
        this.shooter = shooter;
        this.swerve = swerve;
        currentColor = new RGBWColor(0, 0, 0);
        backCandle.setColor(0, 7, new RGBWColor(0, 0, 255, 255));
        frontCandle.setColor(0, 7, new RGBWColor(0, 0, 255, 255));
    }

    public void setColor(RGBWColor color) {
        if (currentColor.Red != color.Red || currentColor.Green != color.Green || currentColor.Blue != color.Blue) {
            backCandle.setColor(Constants.startIndex, Constants.endIndex, color);
            currentColor = color;
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput("LED/Color", new int[] {currentColor.Red, currentColor.Green, currentColor.Blue});
        if (shooter.getFlyVelocity() > 10) {
            if (shooter.atTargetSpeed()) {
                setColor(new RGBWColor(255, 0, 0));
            } else {
                setColor(new RGBWColor(0, 255, 0));
            }
        } else {
            setColor(new RGBWColor());
        }
    }
}
