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
        this.rollerMotor.setInverted(Constants.motorInverted);
        this.rollerMotor.setStatorCurrentLimit(Constants.statorCurrentLimit);
        this.rollerMotor.setSupplyCurrentLimit(Constants.supplyCurrentLimit);
        this.rollerMotor.setSupplyCurrentLowerLimit(Constants.supplyCurrentLowerLimit);
        this.rollerMotor.setSupplyCurrentLowerTime(Constants.supplyCurrentLowerTime);
        this.rollerMotor.setGearRatio(Constants.gearRatio);
        rollerState = RollerState.ROLLER_STOP;
    }

    public static class Constants {
        // CAN device ID for the intake motor controller
        public static final int motorId = 0;
        // Whether to flip motor direction (true means reverse forward/backward)
        public static final boolean motorInverted = false;

        public static final double statorCurrentLimit = 60; // (amps) limit on motor torque output for intake loads
        public static final double supplyCurrentLimit = 50; // (amps) normal current limit pulled from battery
        public static final double supplyCurrentLowerLimit = 35; // (amps) reduce to this if over limit for some time
        public static final double supplyCurrentLowerTime = 0.3; // (seconds) time before lowering current limit
        public static final double gearRatio = 1; // Ratio of motor rotations to arm rotations (unitless)
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
