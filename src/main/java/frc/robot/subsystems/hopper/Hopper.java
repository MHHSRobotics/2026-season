package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.io.MotorIO;

public class Hopper extends SubsystemBase {
    public static enum RollerState {
        ROLLER_STOP,
        ROLLER_FORWARD,
        ROLLER_REVERSE
    };

    private MotorIO rollerMotor;
    private RollerState rollerState;

    public Hopper(MotorIO rollerMotor) {
        this.rollerMotor = rollerMotor;
        this.rollerMotor.setInverted(false);

        rollerState = RollerState.ROLLER_STOP;
    }

    public void setRollerForward() {
        // Sets the roller motor to full velocity forward towards the shooter
        rollerMotor.setDutyCycle(1);
        rollerState = RollerState.ROLLER_FORWARD;
    }

    public void setRollerReverse() {
        // Sets the roller motor to full velocity reversed towards the intake
        rollerMotor.setDutyCycle(-1);
        rollerState = RollerState.ROLLER_REVERSE;
    }

    public void setRollerStop() {
        // Sets the roller motor to zero velocity
        rollerMotor.setDutyCycle(0);
        rollerState = RollerState.ROLLER_STOP;
    }

    public RollerState getRollerState() {
        return rollerState;
    }
}
