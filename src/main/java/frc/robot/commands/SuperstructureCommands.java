package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotPositions;

// Commands for going to positions that require arm, elevator, and wrist to move
public class SuperstructureCommands {
    private ArmCommands armCmds;
    private ElevatorCommands elevatorCmds;
    private WristCommands wristCmds;

    public SuperstructureCommands(ArmCommands armCmds, ElevatorCommands elevatorCmds, WristCommands wristCmds) {
        this.armCmds = armCmds;
        this.elevatorCmds = elevatorCmds;
        this.wristCmds = wristCmds;
    }

    public Command defaultPosition() {
        return new SequentialCommandGroup(
                armCmds.setGoal(() -> RobotPositions.armDefault),
                elevatorCmds.setGoal(() -> RobotPositions.elevatorDefault),
                wristCmds.setGoal(() -> RobotPositions.wristDefault));
    }

    public Command sourcePosition() {
        return new SequentialCommandGroup(
                armCmds.setGoal(() -> RobotPositions.armSource),
                elevatorCmds.setGoal(() -> RobotPositions.elevatorSource),
                wristCmds.setGoal(() -> RobotPositions.wristSource));
    }

    public Command L1Position() {
        return new SequentialCommandGroup(
                armCmds.setGoal(() -> RobotPositions.armL1),
                elevatorCmds.setGoal(() -> RobotPositions.elevatorL1),
                wristCmds.setGoal(() -> RobotPositions.wristL1));
    }

    public Command L2Position() {
        return new SequentialCommandGroup(
                armCmds.setGoal(() -> RobotPositions.armL2),
                elevatorCmds.setGoal(() -> RobotPositions.elevatorL2),
                wristCmds.setGoal(() -> RobotPositions.wristL2));
    }

    public Command L3Position() {
        return new SequentialCommandGroup(
                armCmds.setGoal(() -> RobotPositions.armL3),
                elevatorCmds.setGoal(() -> RobotPositions.elevatorL3),
                wristCmds.setGoal(() -> RobotPositions.wristL3));
    }

    public Command L4Position() {
        return new SequentialCommandGroup(
                armCmds.setGoal(() -> RobotPositions.armL4),
                elevatorCmds.setGoal(() -> RobotPositions.elevatorL4),
                wristCmds.setGoal(() -> RobotPositions.wristL4));
    }

    public Command lowAlgaePosition() {
        return new SequentialCommandGroup(
                armCmds.setGoal(() -> RobotPositions.armLowAlgae),
                elevatorCmds.setGoal(() -> RobotPositions.elevatorLowAlgae),
                wristCmds.setGoal(() -> RobotPositions.wristLowAlgae));
    }

    public Command highAlgaePosition() {
        return new SequentialCommandGroup(
                armCmds.setGoal(() -> RobotPositions.armHighAlgae),
                elevatorCmds.setGoal(() -> RobotPositions.elevatorHighAlgae),
                wristCmds.setGoal(() -> RobotPositions.wristHighAlgae));
    }
}
