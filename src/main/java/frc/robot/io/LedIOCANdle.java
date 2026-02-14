package frc.robot.io;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.Constants;

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
    }

    @Override
    public void setColor(int startIndex, int endIndex, RGBWColor color) {
        candle.setControl(new SolidColor(startIndex, endIndex).withColor(color));
    }
}
