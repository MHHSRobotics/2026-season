package frc.robot.io;

import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;

public class LedIOCANdle extends LedIO {
    private CANdle candle;

    private int id;
    private boolean disconnected = false;

    public LedIOCANdle(String name, String logPath, int id, CANBus bus) {
        super(name, logPath);
        candle = new CANdle(id, bus);

        this.id = id;
    }

    public LedIOCANdle(String name, String logPath, int id) {
        this(name, logPath, id, Constants.defaultBus);
    }

    public int getId() {
        return id;
    }

    @Override
    public void update() {
        inputs.connected = disconnected ? false : candle.isConnected();

        super.update();
    }

    @Override
    public void setColor(int startIndex, int endIndex, RGBWColor color) {
        candle.setControl(new SolidColor(startIndex, endIndex).withColor(color));
    }

    @Override
    public void disconnect() {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method disconnect on " + getName(), AlertType.kWarning);
            return;
        }
        disconnected = true;
    }
}
