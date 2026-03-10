package frc.robot.io;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;

/**
 * Controller wrapper that provides generic button names across PS5, PS4, and Xbox controllers.
 * Detects controller type from button count: PS5=15, PS4=14, Xbox=10.
 *
 * All triggers resolve button numbers dynamically so that swapping controllers at runtime works.
 */
public class GameController extends CommandGenericHID {

    public enum ControllerType {
        PS5,
        PS4,
        XBOX,
        UNKNOWN
    }

    private final String name;
    private ControllerType type = ControllerType.UNKNOWN;

    public GameController(int port, String name) {
        super(port);
        this.name = name;
    }

    /** Detect controller type from button count. Call this every loop from periodic(). */
    public void detectType() {
        int buttonCount = getHID().getButtonCount();
        ControllerType newType;
        switch (buttonCount) {
            case 15:
                newType = ControllerType.PS5;
                break;
            case 14:
                newType = ControllerType.PS4;
                break;
            case 10:
                newType = ControllerType.XBOX;
                break;
            default:
                newType = ControllerType.UNKNOWN;
                break;
        }
        if (newType != type) {
            type = newType;
            if (type == ControllerType.UNKNOWN && buttonCount > 0) {
                DriverStation.reportWarning(
                        "Unknown controller '" + name + "' on port " + getHID().getPort() + " with " + buttonCount
                                + " buttons",
                        false);
            }
        }
        Logger.recordOutput("Controllers/" + name + "/Type", type.toString());
    }

    public ControllerType getControllerType() {
        return type;
    }

    // --- Dynamic trigger helper ---
    // button() from CommandGenericHID binds to a fixed button number.
    // We need triggers that re-evaluate the button number each loop.

    private Trigger dynamicButton(java.util.function.IntSupplier buttonSupplier) {
        return new Trigger(() -> getHID().getRawButton(buttonSupplier.getAsInt()));
    }

    // --- Face buttons (south/east/north/west) ---

    private int southButton() {
        return type == ControllerType.XBOX ? 1 : 2; // A / Cross
    }

    private int eastButton() {
        return type == ControllerType.XBOX ? 2 : 3; // B / Circle
    }

    private int westButton() {
        return type == ControllerType.XBOX ? 3 : 1; // X / Square
    }

    private int northButton() {
        return 4; // Y / Triangle — same on all
    }

    public Trigger south() {
        return dynamicButton(this::southButton);
    }

    public Trigger east() {
        return dynamicButton(this::eastButton);
    }

    public Trigger west() {
        return dynamicButton(this::westButton);
    }

    public Trigger north() {
        return dynamicButton(this::northButton);
    }

    // --- Bumpers ---

    public Trigger leftBumper() {
        return button(5); // L1 / LB — same on all
    }

    public Trigger rightBumper() {
        return button(6); // R1 / RB — same on all
    }

    // --- Trigger buttons (digital press) ---

    /** Left trigger as a digital button. On Xbox, uses axis threshold; on PS, uses button 7. */
    public Trigger leftTrigger() {
        return new Trigger(() -> {
            if (type == ControllerType.XBOX) {
                return getHID().getRawAxis(2) > 0.5;
            }
            return getHID().getRawButton(7);
        });
    }

    /** Right trigger as a digital button. On Xbox, uses axis threshold; on PS, uses button 8. */
    public Trigger rightTrigger() {
        return new Trigger(() -> {
            if (type == ControllerType.XBOX) {
                return getHID().getRawAxis(3) > 0.5;
            }
            return getHID().getRawButton(8);
        });
    }

    // --- Menu buttons ---

    private int leftMenuButton() {
        return type == ControllerType.XBOX ? 7 : 9; // Back-View / Create-Share
    }

    private int rightMenuButton() {
        return type == ControllerType.XBOX ? 8 : 10; // Start-Menu / Options
    }

    public Trigger leftMenu() {
        return dynamicButton(this::leftMenuButton);
    }

    public Trigger rightMenu() {
        return dynamicButton(this::rightMenuButton);
    }

    // --- Stick buttons ---

    private int leftStickButton() {
        return type == ControllerType.XBOX ? 9 : 11;
    }

    private int rightStickButton() {
        return type == ControllerType.XBOX ? 10 : 12;
    }

    public Trigger leftStick() {
        return dynamicButton(this::leftStickButton);
    }

    public Trigger rightStick() {
        return dynamicButton(this::rightStickButton);
    }

    // --- Touchpad (PS only) ---

    /** Touchpad button. Only available on PS4/PS5 controllers. Returns never-true on Xbox. */
    public Trigger touchpad() {
        return new Trigger(() -> {
            if (type == ControllerType.PS5 || type == ControllerType.PS4) {
                return getHID().getRawButton(14);
            }
            return false;
        });
    }

    // --- Axes ---

    private int rightXAxis() {
        return type == ControllerType.XBOX ? 4 : 2;
    }

    private int rightYAxis() {
        return type == ControllerType.XBOX ? 5 : 5;
    }

    public double getLeftX() {
        return getRawAxis(0);
    }

    public double getLeftY() {
        return getRawAxis(1);
    }

    public double getRightX() {
        return getRawAxis(rightXAxis());
    }

    public double getRightY() {
        return getRawAxis(rightYAxis());
    }
}
