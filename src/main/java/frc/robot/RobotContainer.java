package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.Constants.Mode;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.HangCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.SuperstructureCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.commands.WristCommands;
import frc.robot.io.CameraIO;
import frc.robot.io.CameraIOPhotonCamera;
import frc.robot.io.EncoderIO;
import frc.robot.io.EncoderIOCANcoder;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIOTalonFX;
import frc.robot.network.RobotPublisher;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorSubsystemSim;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.GyroSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleSim;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.swerve.VisionSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristSim;
import frc.robot.util.Alerts;

public class RobotContainer {
    private Arm arm;
    private Elevator elevator;
    private Wrist wrist;
    private Hang hang;
    private Intake intake;
    private Swerve swerve;

    private ArmCommands armCommands;
    private ElevatorCommands elevatorCommands;
    private WristCommands wristCommands;
    private HangCommands hangCommands;
    private IntakeCommands intakeCommands;
    private SwerveCommands swerveCommands;

    private SuperstructureCommands ssCommands;

    // Main drive controller
    private final CommandPS5Controller controller = new CommandPS5Controller(0);

    // Manual controller for subsystems
    private final CommandPS5Controller manualController = new CommandPS5Controller(1);

    // Test controller for controlling one subsystem at a time
    private final CommandPS5Controller testController = new CommandPS5Controller(2);

    private LoggedDashboardChooser<String> testControllerChooser;
    private LoggedDashboardChooser<String> testControllerManual;
    private LoggedDashboardChooser<String> autoChooser;

    // Publishes all robot data to AdvantageScope
    private RobotPublisher publisher;

    private Alert controllerDisconnected = new Alert("Drive controller is disconnected", AlertType.kWarning);
    private Alert manualDisconnected = new Alert("Manual controller is disconnected", AlertType.kWarning);

    public RobotContainer() {
        // Initialize all the IO objects, subsystems, and mechanism simulators
        initSubsystems();

        // Initialize command classes
        initCommands();

        // Add controller bindings
        configureBindings();

        // Configure bindings for manual controller
        configureManualBindings();

        // Configure bindings for test controller when not in match
        if (!DriverStation.isFMSAttached()) {
            configureTestBindings();
        }

        // Set up the auto chooser
        configureAutoChooser();

        // Initialize the publisher
        publisher = new RobotPublisher(arm, wrist, intake, elevator, hang, swerve);
    }

    private void initSubsystems() {
        // Initialize subsystems in order: arm, elevator, wrist, intake, hang, swerve
        // Each subsystem is created immediately after its motor/encoder initialization
        if (Constants.armEnabled) {
            // Initialize arm motor and encoder
            MotorIO armMotor;
            EncoderIO armEncoder;
            switch (Constants.currentMode) {
                case REAL:
                case SIM:
                    armMotor =
                            new MotorIOTalonFX(Arm.Constants.motorId, Constants.defaultBus, "arm motor", "Arm/Motor");
                    armEncoder = new EncoderIOCANcoder(
                            Arm.Constants.encoderId, Constants.defaultBus, "arm encoder", "Arm/Encoder");
                    break;
                default:
                    armMotor = new MotorIO("arm motor", "Arm/Motor");
                    armEncoder = new EncoderIO("arm encoder", "Arm/Encoder");
                    break;
            }
            // Create arm subsystem
            arm = new Arm(armMotor, armEncoder);

            if (Constants.currentMode == Mode.SIM) {
                new ArmSim(armMotor, armEncoder);
            }

            if (Constants.wristEnabled) {
                // Initialize wrist motor and encoder
                MotorIO wristMotor;
                EncoderIO wristEncoder;
                switch (Constants.currentMode) {
                    case REAL:
                    case SIM:
                        wristMotor = new MotorIOTalonFX(
                                Wrist.Constants.motorId, Constants.defaultBus, "wrist motor", "Wrist/Motor");
                        wristEncoder = new EncoderIOCANcoder(
                                Wrist.Constants.encoderId, Constants.defaultBus, "wrist encoder", "Wrist/Encoder");
                        break;
                    default:
                        wristMotor = new MotorIO("wrist motor", "Wrist/Motor");
                        wristEncoder = new EncoderIO("wrist encoder", "Wrist/Encoder");
                        break;
                }
                // Create wrist subsystem
                wrist = new Wrist(wristMotor, wristEncoder, armMotor);

                if (Constants.currentMode == Mode.SIM) {
                    new WristSim(wristMotor, wristEncoder);
                }
            }
        }

        if (Constants.elevatorEnabled) {
            // Initialize elevator motors and encoder
            MotorIO elevatorLeftMotor;
            MotorIO elevatorRightMotor;
            EncoderIO elevatorEncoder;
            switch (Constants.currentMode) {
                case REAL:
                case SIM:
                    elevatorLeftMotor = new MotorIOTalonFX(
                            Elevator.Constants.leftMotorId,
                            Constants.defaultBus,
                            "elevator left motor",
                            "Elevator/LeftMotor");
                    elevatorRightMotor = new MotorIOTalonFX(
                            Elevator.Constants.rightMotorId,
                            Constants.defaultBus,
                            "elevator right motor",
                            "Elevator/RightMotor");
                    elevatorEncoder = new EncoderIOCANcoder(
                            Elevator.Constants.encoderId, Constants.defaultBus, "elevator encoder", "Elevator/Encoder");
                    break;
                default:
                    elevatorLeftMotor = new MotorIO("elevator left motor", "Elevator/LeftMotor");
                    elevatorRightMotor = new MotorIO("elevator right motor", "Elevator/RightMotor");
                    elevatorEncoder = new EncoderIO("elevator encoder", "Elevator/Encoder");
                    break;
            }
            // Create elevator subsystem
            elevator = new Elevator(elevatorLeftMotor, elevatorRightMotor, elevatorEncoder);

            if (Constants.currentMode == Mode.SIM) {
                new ElevatorSubsystemSim(elevatorLeftMotor, elevatorRightMotor, elevatorEncoder);
            }
        }

        if (Constants.intakeEnabled) {
            // Initialize intake motor
            MotorIO intakeMotor;
            switch (Constants.currentMode) {
                case REAL:
                case SIM:
                    intakeMotor = new MotorIOTalonFX(
                            Intake.Constants.motorId, Constants.defaultBus, "intake motor", "Intake/Motor");
                    break;
                default:
                    intakeMotor = new MotorIO("intake motor", "Intake/Motor");
                    break;
            }
            // Create intake subsystem
            intake = new Intake(intakeMotor);
        }

        if (Constants.hangEnabled) {
            // Initialize hang motor
            MotorIO hangMotor;
            switch (Constants.currentMode) {
                case REAL:
                case SIM:
                    hangMotor = new MotorIOTalonFX(
                            Hang.Constants.motorId, Constants.defaultBus, "hang motor", "Hang/Motor");
                    break;
                default:
                    hangMotor = new MotorIO("hang motor", "Hang/Motor");
                    break;
            }
            // Create hang subsystem
            hang = new Hang(hangMotor);
        }

        if (Constants.swerveEnabled) {
            // Initialize swerve motors, encoders, and gyro
            MotorIO flDriveMotor, flAngleMotor, frDriveMotor, frAngleMotor;
            MotorIO blDriveMotor, blAngleMotor, brDriveMotor, brAngleMotor;
            EncoderIO flEncoder, frEncoder, blEncoder, brEncoder;
            GyroIO gyro;
            switch (Constants.currentMode) {
                case REAL:
                case SIM:
                    flDriveMotor = new MotorIOTalonFX(
                            TunerConstants.FrontLeft.DriveMotorId,
                            Constants.swerveBus,
                            "front left drive motor",
                            "Swerve/FrontLeft/Drive");
                    flAngleMotor = new MotorIOTalonFX(
                            TunerConstants.FrontLeft.SteerMotorId,
                            Constants.swerveBus,
                            "front left angle motor",
                            "Swerve/FrontLeft/Steer");
                    flEncoder = new EncoderIOCANcoder(
                            TunerConstants.FrontLeft.EncoderId,
                            Constants.swerveBus,
                            "front left encoder",
                            "Swerve/FrontLeft/Encoder");

                    frDriveMotor = new MotorIOTalonFX(
                            TunerConstants.FrontRight.DriveMotorId,
                            Constants.swerveBus,
                            "front right drive motor",
                            "Swerve/FrontRight/Drive");
                    frAngleMotor = new MotorIOTalonFX(
                            TunerConstants.FrontRight.SteerMotorId,
                            Constants.swerveBus,
                            "front right angle motor",
                            "Swerve/FrontRight/Steer");
                    frEncoder = new EncoderIOCANcoder(
                            TunerConstants.FrontRight.EncoderId,
                            Constants.swerveBus,
                            "front right encoder",
                            "Swerve/FrontRight/Encoder");

                    blDriveMotor = new MotorIOTalonFX(
                            TunerConstants.BackLeft.DriveMotorId,
                            Constants.swerveBus,
                            "back left drive motor",
                            "Swerve/BackLeft/Drive");
                    blAngleMotor = new MotorIOTalonFX(
                            TunerConstants.BackLeft.SteerMotorId,
                            Constants.swerveBus,
                            "back left angle motor",
                            "Swerve/BackLeft/Steer");
                    blEncoder = new EncoderIOCANcoder(
                            TunerConstants.BackLeft.EncoderId,
                            Constants.swerveBus,
                            "back left encoder",
                            "Swerve/BackLeft/Encoder");

                    brDriveMotor = new MotorIOTalonFX(
                            TunerConstants.BackRight.DriveMotorId,
                            Constants.swerveBus,
                            "back right drive motor",
                            "Swerve/BackRight/Drive");
                    brAngleMotor = new MotorIOTalonFX(
                            TunerConstants.BackRight.SteerMotorId,
                            Constants.swerveBus,
                            "back right angle motor",
                            "Swerve/BackRight/Steer");
                    brEncoder = new EncoderIOCANcoder(
                            TunerConstants.BackRight.EncoderId,
                            Constants.swerveBus,
                            "back right encoder",
                            "Swerve/BackRight/Encoder");

                    gyro = new GyroIOPigeon(
                            TunerConstants.DrivetrainConstants.Pigeon2Id, Constants.swerveBus, "gyro", "Swerve/Gyro");
                    break;
                default:
                    flDriveMotor = new MotorIO("front left drive motor", "Swerve/FrontLeft/Drive");
                    flAngleMotor = new MotorIO("front left angle motor", "Swerve/FrontLeft/Steer");
                    flEncoder = new EncoderIO("front left encoder", "Swerve/FrontLeft/Encoder");

                    frDriveMotor = new MotorIO("front right drive motor", "Swerve/FrontRight/Drive");
                    frAngleMotor = new MotorIO("front right angle motor", "Swerve/FrontRight/Steer");
                    frEncoder = new EncoderIO("front right encoder", "Swerve/FrontRight/Encoder");

                    blDriveMotor = new MotorIO("back left drive motor", "Swerve/BackLeft/Drive");
                    blAngleMotor = new MotorIO("back left angle motor", "Swerve/BackLeft/Steer");
                    blEncoder = new EncoderIO("back left encoder", "Swerve/BackLeft/Encoder");

                    brDriveMotor = new MotorIO("back right drive motor", "Swerve/BackRight/Drive");
                    brAngleMotor = new MotorIO("back right angle motor", "Swerve/BackRight/Steer");
                    brEncoder = new EncoderIO("back right encoder", "Swerve/BackRight/Encoder");

                    gyro = new GyroIO("gyro", "Swerve/Gyro");
                    break;
            }
            // Create swerve subsystem
            SwerveModule fl = new SwerveModule(flDriveMotor, flAngleMotor, flEncoder, TunerConstants.FrontLeft);
            SwerveModule fr = new SwerveModule(frDriveMotor, frAngleMotor, frEncoder, TunerConstants.FrontRight);
            SwerveModule bl = new SwerveModule(blDriveMotor, blAngleMotor, blEncoder, TunerConstants.BackLeft);
            SwerveModule br = new SwerveModule(brDriveMotor, brAngleMotor, brEncoder, TunerConstants.BackRight);

            swerve = new Swerve(gyro, fl, fr, bl, br);
            if (Constants.currentMode == Mode.SIM) {
                new SwerveModuleSim(flDriveMotor, flAngleMotor, flEncoder, TunerConstants.FrontLeft);
                new SwerveModuleSim(frDriveMotor, frAngleMotor, frEncoder, TunerConstants.FrontRight);
                new SwerveModuleSim(blDriveMotor, blAngleMotor, blEncoder, TunerConstants.BackLeft);
                new SwerveModuleSim(brDriveMotor, brAngleMotor, brEncoder, TunerConstants.BackRight);

                new GyroSim(gyro);
            }
        }

        if (Constants.visionEnabled) {
            CameraIO brat;
            CameraIO blat;
            switch (Constants.currentMode) {
                case REAL:
                case SIM:
                    brat = new CameraIOPhotonCamera("BackRight_AT", "Vision/BRAT", Swerve.VisionConstants.bratPose, 60);
                    blat = new CameraIOPhotonCamera("BackLeft_AT", "Vision/BLAT", Swerve.VisionConstants.blatPose, 60);
                    break;
                default:
                    brat = new CameraIO("BackRight_AT", "Vision/BRAT");
                    blat = new CameraIO("BackLeft_AT", "Vision/BLAT");
                    break;
            }
            swerve.addCameraSource(brat);
            swerve.addCameraSource(blat);
            if (Constants.currentMode == Mode.SIM) {
                new VisionSim(swerve.getCameras(), swerve);
            }
        }
    }

    private void initCommands() {
        armCommands = new ArmCommands(arm);
        elevatorCommands = new ElevatorCommands(elevator);
        wristCommands = new WristCommands(wrist);
        hangCommands = new HangCommands(hang);
        intakeCommands = new IntakeCommands(intake);
        swerveCommands = new SwerveCommands(swerve);
        ssCommands = new SuperstructureCommands(armCommands, elevatorCommands, wristCommands);
    }

    private void configureBindings() {
        /* ---- Main controller bindings ---- */
        controller.triangle().onTrue(ssCommands.L4Position());
        controller.square().onTrue(ssCommands.L3Position());
        controller.circle().onTrue(ssCommands.L2Position());
        controller.cross().onTrue(ssCommands.L1Position());
        controller.options().onTrue(ssCommands.defaultPosition());

        // controller.R2().onTrue(ssCommands.lowAlgaePosition());
        controller.R1().onTrue(ssCommands.sourcePosition());

        controller.L1().onTrue(intakeCommands.intake()).onFalse(intakeCommands.stop());
        controller.L2().onTrue(intakeCommands.outtake()).onFalse(intakeCommands.stop());

        controller.povUp().onTrue(hangCommands.extendUp()).onFalse(hangCommands.stop());
        controller.povDown().onTrue(hangCommands.retractDown()).onFalse(hangCommands.stop());

        controller.create().onTrue(swerveCommands.resetGyro());

        controller.povLeft().onTrue(swerveCommands.alignToSide(0));

        controller.povRight().onTrue(swerveCommands.alignToSide(1));

        controller
                .axisMagnitudeGreaterThan(0, Swerve.Constants.moveDeadband)
                .or(controller.axisMagnitudeGreaterThan(1, Swerve.Constants.moveDeadband))
                .or(controller.axisMagnitudeGreaterThan(2, Swerve.Constants.turnDeadband))
                .onTrue(swerveCommands.drive(
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX(),
                        () -> Swerve.Constants.swerveFieldCentric.get()));

        // Cancel all commands
        controller.PS().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance()
                .cancelAll()));
    }

    private void configureTestBindings() {
        /* ---- Test controller bindings ---- */
        testControllerChooser = new LoggedDashboardChooser<>("Test/Subsystem");
        testControllerChooser.addOption("Arm", "Arm");
        testControllerChooser.addOption("Elevator", "Elevator");
        testControllerChooser.addOption("Wrist", "Wrist");
        testControllerChooser.addOption("Hang", "Hang");
        testControllerChooser.addOption("Intake", "Intake");
        testControllerChooser.addOption("Swerve", "Swerve");

        testControllerManual = new LoggedDashboardChooser<>("Test/Type");
        testControllerManual.addOption("Manual", "Manual");
        testControllerManual.addOption("PID", "PID");

        // Test controller swerve control for convenience
        testController
                .axisMagnitudeGreaterThan(0, Swerve.Constants.moveDeadband)
                .or(testController.axisMagnitudeGreaterThan(1, Swerve.Constants.moveDeadband))
                .or(testController.axisMagnitudeGreaterThan(2, Swerve.Constants.turnDeadband))
                .onTrue(swerveCommands.drive(
                        () -> -testController.getLeftY(),
                        () -> -testController.getLeftX(),
                        () -> -testController.getRightX(),
                        () -> Swerve.Constants.swerveFieldCentric.get()));

        // Manual duty cycle forward test
        testController
                .cross()
                .and(() -> testControllerManual.get().equals("Manual"))
                .and(() -> testControllerChooser.get().equals("Arm"))
                .onTrue(armCommands.setSpeed(0.2))
                .onFalse(armCommands.stop());
        testController
                .cross()
                .and(() -> testControllerManual.get().equals("Manual"))
                .and(() -> testControllerChooser.get().equals("Elevator"))
                .onTrue(elevatorCommands.setSpeed(0.2))
                .onFalse(elevatorCommands.stop());
        testController
                .cross()
                .and(() -> testControllerManual.get().equals("Manual"))
                .and(() -> testControllerChooser.get().equals("Wrist"))
                .onTrue(wristCommands.setSpeed(0.2))
                .onFalse(wristCommands.stop());
        testController
                .cross()
                .and(() -> testControllerManual.get().equals("Manual"))
                .and(() -> testControllerChooser.get().equals("Hang"))
                .onTrue(hangCommands.setSpeed(0.2))
                .onFalse(hangCommands.stop());
        testController
                .cross()
                .and(() -> testControllerManual.get().equals("Manual"))
                .and(() -> testControllerChooser.get().equals("Intake"))
                .onTrue(intakeCommands.setSpeed(0.2))
                .onFalse(intakeCommands.stop());
        testController
                .cross()
                .and(() -> testControllerManual.get().equals("Manual"))
                .and(() -> testControllerChooser.get().equals("Swerve"))
                .onTrue(swerveCommands.setSpeed(0.2, 0, 0))
                .onFalse(swerveCommands.stop());

        // Manual duty cycle backward test
        testController
                .circle()
                .and(() -> testControllerManual.get().equals("Manual"))
                .and(() -> testControllerChooser.get().equals("Arm"))
                .onTrue(armCommands.setSpeed(-0.2))
                .onFalse(armCommands.stop());
        testController
                .circle()
                .and(() -> testControllerManual.get().equals("Manual"))
                .and(() -> testControllerChooser.get().equals("Elevator"))
                .onTrue(elevatorCommands.setSpeed(-0.2))
                .onFalse(elevatorCommands.stop());
        testController
                .circle()
                .and(() -> testControllerManual.get().equals("Manual"))
                .and(() -> testControllerChooser.get().equals("Wrist"))
                .onTrue(wristCommands.setSpeed(-0.2))
                .onFalse(wristCommands.stop());
        testController
                .circle()
                .and(() -> testControllerManual.get().equals("Manual"))
                .and(() -> testControllerChooser.get().equals("Hang"))
                .onTrue(hangCommands.setSpeed(-0.2))
                .onFalse(hangCommands.stop());
        testController
                .circle()
                .and(() -> testControllerManual.get().equals("Manual"))
                .and(() -> testControllerChooser.get().equals("Intake"))
                .onTrue(intakeCommands.setSpeed(-0.2))
                .onFalse(intakeCommands.stop());
        testController
                .circle()
                .and(() -> testControllerManual.get().equals("Manual"))
                .and(() -> testControllerChooser.get().equals("Swerve"))
                .onTrue(swerveCommands.setSpeed(-0.2, 0, 0))
                .onFalse(swerveCommands.stop());

        // PID down test
        testController
                .cross()
                .and(() -> testControllerManual.get().equals("PID"))
                .and(() -> testControllerChooser.get().equals("Arm"))
                .onTrue(armCommands.setGoal(0));
        testController
                .cross()
                .and(() -> testControllerManual.get().equals("PID"))
                .and(() -> testControllerChooser.get().equals("Elevator"))
                .onTrue(elevatorCommands.setGoal(0.1));
        testController
                .cross()
                .and(() -> testControllerManual.get().equals("PID"))
                .and(() -> testControllerChooser.get().equals("Wrist"))
                .onTrue(wristCommands.setGoal(0));
        testController
                .cross()
                .and(() -> testControllerManual.get().equals("PID"))
                .and(() -> testControllerChooser.get().equals("Swerve"))
                .onTrue(swerveCommands.alignToSide(0)); // Align to nearest left reef

        // PID up test
        testController
                .circle()
                .and(() -> testControllerManual.get().equals("PID"))
                .and(() -> testControllerChooser.get().equals("Arm"))
                .onTrue(armCommands.setGoal(1));
        testController
                .circle()
                .and(() -> testControllerManual.get().equals("PID"))
                .and(() -> testControllerChooser.get().equals("Elevator"))
                .onTrue(elevatorCommands.setGoal(0.5));
        testController
                .circle()
                .and(() -> testControllerManual.get().equals("PID"))
                .and(() -> testControllerChooser.get().equals("Wrist"))
                .onTrue(wristCommands.setGoal(1));
        testController
                .circle()
                .and(() -> testControllerManual.get().equals("PID"))
                .and(() -> testControllerChooser.get().equals("Swerve"))
                .onTrue(swerveCommands.alignToSide(1)); // Align to nearest right reef
    }

    // Bindings for manual control of each of the subsystems
    public void configureManualBindings() {
        // Square + circle control arm
        manualController.square().whileTrue(new RepeatCommand(armCommands.changeGoal(0.05)));
        manualController.circle().whileTrue(new RepeatCommand(armCommands.changeGoal(-0.05)));

        // Triangle + cross control elevator
        manualController.triangle().whileTrue(new RepeatCommand(elevatorCommands.changeGoal(0.05)));
        manualController.cross().whileTrue(new RepeatCommand(elevatorCommands.changeGoal(-0.05)));

        // POV up + down control wrist
        manualController.povDown().whileTrue(new RepeatCommand(wristCommands.changeGoal(0.05)));
        manualController.povUp().whileTrue(new RepeatCommand(wristCommands.changeGoal(-0.05)));

        // Intake/outtake controls
        manualController.L1().onTrue(intakeCommands.intake()).onFalse(intakeCommands.stop());
        manualController.L2().onTrue(intakeCommands.outtake()).onFalse(intakeCommands.stop());

        // Hang controls
        manualController.povLeft().onTrue(hangCommands.extendUp()).onFalse(hangCommands.stop());
        manualController.povRight().onTrue(hangCommands.retractDown()).onFalse(hangCommands.stop());

        manualController.options().onTrue(ssCommands.defaultPosition());
    }

    // Refresh drive and manual controller disconnect alerts
    public void refreshControllerAlerts() {
        controllerDisconnected.set(!controller.isConnected());
        manualDisconnected.set(!manualController.isConnected());
    }

    public void configureAutoChooser() {
        autoChooser = new LoggedDashboardChooser<>("AutoSelection");
        autoChooser.addOption("Left", "Left");
        autoChooser.addOption("Right", "Right");
        autoChooser.addDefaultOption("Leave", "Leave");
    }

    public Command getAutonomousCommand() {
        if (autoChooser.get().equals("Leave")) {
            return swerveCommands
                    .setPositionOutput(-2, 0)
                    .andThen(new WaitCommand(3))
                    .andThen(swerveCommands.setPositionOutput(0, 0));
        } else if (autoChooser.get().equals("Left") || autoChooser.get().equals("Right")) {
            return ssCommands
                    .defaultPosition()
                    .andThen(new WaitCommand(0.5))
                    .andThen(swerveCommands.alignToSide(autoChooser.get().equals("Right") ? 1 : 0))
                    .andThen(new WaitUntilCommand(
                            () -> swerve.getRotationError() < 0.1 && swerve.getTranslationError() < 0.1))
                    .andThen(ssCommands.L4Position())
                    .andThen(new WaitCommand(3))
                    .andThen(intakeCommands.outtake())
                    .andThen(new WaitCommand(0.2))
                    .andThen(intakeCommands.stop());
        } else {
            Alerts.create("Unknown auto specified", AlertType.kWarning);
            return new InstantCommand();
        }
    }

    public void periodic() {
        // Publish 3D robot data
        publisher.publish();

        // Enable alerts for controller disconnects
        refreshControllerAlerts();
    }
}
