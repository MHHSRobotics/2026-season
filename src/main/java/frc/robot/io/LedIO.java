package frc.robot.io;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.signals.RGBWColor;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;

public class LedIO {
    @AutoLog
    public static class LedIOInputs {
        public boolean connected;
    }

    private String logPath;

    private String name;

    // Alert objects to show gyro problems on the dashboard
    private Alert disconnectAlert;

    public LedIO(String name, String logPath) {
        this.name = name;
        this.logPath = logPath;

        // Create alerts with descriptive names for this gyro
        disconnectAlert = new Alert("The " + name + " is disconnected", AlertType.kError);
    }

    public String getName() {
        return name;
    }

    protected LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

    // Find out the latest values from the gyro and store them in inputs
    public void update() {
        // Log the inputs to AdvantageKit
        Logger.processInputs(logPath, inputs);

        // Update alerts based on the current gyro status (this runs after subclass updates inputs)
        // Only update alerts if they've been created (setName() was called)
        disconnectAlert.set(!inputs.connected);
    }

    public LedIOInputs getInputs() {
        return inputs;
    }

    private void unsupportedFeature() {
        if (Constants.currentMode != Mode.REPLAY) {
            Alerts.create("An unsupported feature was used on " + getName(), AlertType.kWarning);
        }
    }

    public void setColor(int startIndex, int endIndex, RGBWColor color) {
        unsupportedFeature();
    }

    public void disconnect() {
        unsupportedFeature();
    }
}
