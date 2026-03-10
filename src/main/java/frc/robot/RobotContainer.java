package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import frc.robot.Constants.Mode;
import frc.robot.commands.HangCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.LEDCommands;
import frc.robot.commands.MultiCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.io.CameraIO;
import frc.robot.io.CameraIOPhotonCamera;
import frc.robot.io.EncoderIO;
import frc.robot.io.EncoderIOCANcoder;
import frc.robot.io.GameController;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon;
import frc.robot.io.LedIO;
import frc.robot.io.LedIOCANdle;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIOTalonFX;
import frc.robot.network.RobotPublisher;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePhysicsSim;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterPhysicsSim;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.swerve.GyroSim;
import frc.robot.subsystems.swerve.SimpleSwerveSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModulePhysicsSim;
import frc.robot.subsystems.swerve.SwerveModuleSim;
import frc.robot.subsystems.swerve.SwervePhysicsSim;
import frc.robot.subsystems.swerve.SwerveRotation;
import frc.robot.subsystems.swerve.SwerveTranslation;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.swerve.VisionSim;
import frc.robot.util.Alerts;
import frc.robot.util.FieldPose2d;
import frc.robot.util.RobotUtils;

public class RobotContainer {
    // Subsystems
    private Swerve swerve;
    private SwerveTranslation swerveTranslation;
    private SwerveRotation swerveRotation;
    private Hang hang;
    private Intake intake;
    private Shooter shooter;
    private LED led;

    private SwerveCommands swerveCommands;
    private HangCommands hangCommands;
    private IntakeCommands intakeCommands;
    private ShooterCommands shooterCommands;
    private LEDCommands ledCommands;

    private MultiCommands multiCommands;

    private final GameController driveController = new GameController(0, "Driver");

    private final GameController operator = new GameController(1, "Operator");

    private final GameController otherController = new GameController(2, "Other");

    private LoggedNetworkBoolean testEnabled;
    private LoggedNetworkNumber testSpeed;
    private LoggedDashboardChooser<String> testSubsystem; // Which subsystem the test controller is applied to
    private LoggedDashboardChooser<String> testType; // Whether to use manual or PID mode for the test controller

    private LoggedDashboardChooser<Command> autoChooser; // Choice of auto

    private RobotPublisher publisher; // Publishes 3D robot data to AdvantageScope for visualization

    // Alerts for disconnected controllers
    private Alert controllerDisconnected = new Alert("Drive controller is disconnected", AlertType.kWarning);
    private Alert operatorDisconnected = new Alert("Operator is disconnected", AlertType.kWarning);

    public RobotContainer() {
        initSubsystems(); // Initialize all the IO objects, subsystems, and mechanism simulators
        initCommands(); // Initialize command classes

        // Configure bindings for test controller when not in match
        if (!DriverStation.isFMSAttached()) {
            configureTestBindings();
        }

        configureAuto(); // Set up the auto names commands and chooser

        configureBindings(); // Add drive controller bindings
        publisher = new RobotPublisher(swerve); // Initialize the 3D data publisher
    }

    private void initSubsystems() {
        // Initialize subsystems in order: arm, elevator, wrist, intake, hang, swerve
        // Each subsystem is created immediately after its motor/encoder initialization

        if (Constants.swerveEnabled) {
            // Create variables for each
            MotorIO flDriveMotor, flAngleMotor, frDriveMotor, frAngleMotor;
            MotorIO blDriveMotor, blAngleMotor, brDriveMotor, brAngleMotor;
            EncoderIO flEncoder, frEncoder, blEncoder, brEncoder;
            GyroIO gyro;
            switch (Constants.currentMode) {
                // If in REAL or SIM mode, use MotorIOTalonFX for motors, EncoderIOCANcoder for encoders, and
                // GyroIOPigeon for the gyro
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
                    // If in REPLAY, use empty MotorIO objects
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
            // Initialize swerve modules
            SwerveModule fl = new SwerveModule(flDriveMotor, flAngleMotor, flEncoder, TunerConstants.FrontLeft);
            SwerveModule fr = new SwerveModule(frDriveMotor, frAngleMotor, frEncoder, TunerConstants.FrontRight);
            SwerveModule bl = new SwerveModule(blDriveMotor, blAngleMotor, blEncoder, TunerConstants.BackLeft);
            SwerveModule br = new SwerveModule(brDriveMotor, brAngleMotor, brEncoder, TunerConstants.BackRight);

            swerve = new Swerve(gyro, fl, fr, bl, br); // Initialize swerve subsystem
            swerveTranslation = new SwerveTranslation();
            swerveRotation = new SwerveRotation();

            if (Constants.visionEnabled) {
                // Create camera variables
                CameraIO frontCam, rightCam;
                switch (Constants.currentMode) {
                    case REAL:
                    case SIM:
                        // If in real bot or sim, use CameraIOPhotonCamera
                        frontCam = new CameraIOPhotonCamera(
                                "FrontCam", "Vision/FrontCam", Swerve.VisionConstants.frontCamPose, 60);
                        rightCam = new CameraIOPhotonCamera(
                                "RightCam", "Vision/RightCam", Swerve.VisionConstants.rightCamPose, 60);
                        // hubRightCam = new CameraIOPhotonCamera(
                        //         "right hub camera", "Vision/RightHubCam", Swerve.VisionConstants.hubRightCamPose,
                        // 60);
                        // hangCam = new CameraIOPhotonCamera(
                        //         "hang camera", "Vision/HangCam", Swerve.VisionConstants.hangCamPose, 60);
                        break;
                    default:
                        // If in replay use an empty CameraIO
                        frontCam = new CameraIO("FrontCam", "Vision/FrontCam");
                        rightCam = new CameraIO("RightCam", "Vision/RightCam");
                        // hubRightCam = new CameraIO("right hub camera", "Vision/RightHubCam");
                        // hangCam = new CameraIO("hang camera", "Vision/HangCam");
                        break;
                }
                // Add cameras to swerve ododmetry
                swerve.addCameraSource(frontCam);
                swerve.addCameraSource(rightCam);
                // swerve.addCameraSource(hubRightCam);
                // swerve.addCameraSource(hangCam);
            }

            // If mode is SIM, start the simulations for swerve modules and gyro
            if (Constants.currentMode == Mode.SIM) {
                if (!Constants.enablePhysicsSim) {
                    SwerveModuleSim[] moduleSims = new SwerveModuleSim[] {
                        new SwerveModuleSim(flDriveMotor, flAngleMotor, flEncoder, TunerConstants.FrontLeft),
                        new SwerveModuleSim(frDriveMotor, frAngleMotor, frEncoder, TunerConstants.FrontRight),
                        new SwerveModuleSim(blDriveMotor, blAngleMotor, blEncoder, TunerConstants.BackLeft),
                        new SwerveModuleSim(brDriveMotor, brAngleMotor, brEncoder, TunerConstants.BackRight)
                    };

                    SimpleSwerveSim swerveSim = new SimpleSwerveSim(moduleSims);

                    new GyroSim(gyro, swerveSim);
                    if (Constants.visionEnabled) {
                        new VisionSim(swerve.getCameras(), swerveSim);
                    }
                } else {
                    new SwerveModulePhysicsSim(flDriveMotor, flAngleMotor, flEncoder, "/MuJoCo/Swerve/FrontLeft");
                    new SwerveModulePhysicsSim(frDriveMotor, frAngleMotor, frEncoder, "/MuJoCo/Swerve/FrontRight");
                    new SwerveModulePhysicsSim(blDriveMotor, blAngleMotor, blEncoder, "/MuJoCo/Swerve/BackLeft");
                    new SwerveModulePhysicsSim(brDriveMotor, brAngleMotor, brEncoder, "/MuJoCo/Swerve/BackRight");

                    SwervePhysicsSim swerveSim = new SwervePhysicsSim("MuJoCo/Swerve/Pose");

                    new GyroSim(gyro, swerveSim);
                    if (Constants.visionEnabled) {
                        new VisionSim(swerve.getCameras(), swerveSim);
                    }
                }
            }
        }

        if (Constants.shooterEnabled) {
            MotorIO feedMotor, flyMotor, flyMotor2;
            switch (Constants.currentMode) {
                // If in REAL or SIM mode, use MotorIOTalonFX for motors, EncoderIOCANcoder for encoders, and
                // GyroIOPigeon for the gyro
                case REAL:
                case SIM:
                    feedMotor = new MotorIOTalonFX(
                            Shooter.Constants.feedMotorId, Constants.defaultBus, "shooter feed motor", "Shooter/Feed");
                    flyMotor = new MotorIOTalonFX(
                            Shooter.Constants.flyMotorId,
                            Constants.defaultBus,
                            "shooter fly motor",
                            "Shooter/Flywheel");
                    flyMotor2 = new MotorIOTalonFX(
                            Shooter.Constants.flyMotorId2,
                            Constants.defaultBus,
                            "shooter fly motor 2",
                            "Shooter/Flywheel2");
                    break;
                default:
                    feedMotor = new MotorIO("shooter feed motor", "Shooter/Feed");
                    flyMotor = new MotorIO("shooter fly motor", "Shooter/Flywheel");
                    flyMotor2 = new MotorIO("shooter fly motor 2", "Shooter/Flywheel2");
                    break;
            }
            shooter = new Shooter(feedMotor, flyMotor, flyMotor2);

            if (Constants.currentMode == Mode.SIM) {
                if (!Constants.enablePhysicsSim) {
                    new ShooterSim(feedMotor, flyMotor, flyMotor2);
                } else {
                    new ShooterPhysicsSim(feedMotor, flyMotor, flyMotor2, "/MuJoCo/Shooter");
                }
            }
        }

        if (Constants.hangEnabled) {
            MotorIO hangMotor;
            EncoderIO hangEncoder;
            switch (Constants.currentMode) {
                case REAL:
                case SIM:
                    hangMotor = new MotorIOTalonFX(
                            Hang.Constants.motorId, Constants.defaultBus, "hang motor", "Hang/Motor");
                    hangEncoder = new EncoderIOCANcoder(
                            Hang.Constants.encoderId, Constants.defaultBus, "hang encoder", "Hang/Encoder");
                    break;
                default:
                    hangMotor = new MotorIO("hang motor", "Hang/Motor");
                    hangEncoder = new EncoderIO("hang encoder", "Hang/Encoder");
                    break;
            }
            hang = new Hang(hangMotor);
        }

        if (Constants.intakeEnabled) {
            MotorIO rollerMotor;
            MotorIO hingeMotor;
            EncoderIO hingeEncoder;
            switch (Constants.currentMode) {
                case REAL:
                case SIM:
                    rollerMotor = new MotorIOTalonFX(
                            Intake.Constants.rollerMotorId,
                            Constants.defaultBus,
                            "intake roller motor",
                            "Intake/Roller");
                    hingeMotor = new MotorIOTalonFX(
                            Intake.Constants.hingeMotorId, Constants.defaultBus, "intake hinge motor", "Intake/Hinge");
                    hingeEncoder = new EncoderIOCANcoder(
                            Intake.Constants.hingeEncoderId, "intake hinge encoder", "Intake/HingeEncoder");
                    break;
                default:
                    rollerMotor = new MotorIO("intake roller motor", "Intake/Roller");
                    hingeMotor = new MotorIO("intake hinge motor", "Intake/Hinge");
                    hingeEncoder = new EncoderIO("intake hinge encoder", "Intake/HingeEncoder");
                    break;
            }
            intake = new Intake(rollerMotor, hingeMotor, hingeEncoder);

            if (Constants.currentMode == Mode.SIM) {
                if (!Constants.physicsSimEnabled) {
                    new IntakeSim(rollerMotor, hingeMotor, hingeEncoder);
                } else {
                    new IntakePhysicsSim(rollerMotor, hingeMotor, hingeEncoder, "/MuJoCo/Intake");
                }
            }
        }

        if (Constants.ledsEnabled) {
            LedIO ledIO;
            switch (Constants.currentMode) {
                case REAL:
                case SIM:
                    ledIO = new LedIOCANdle("leds", "LED", LED.Constants.id);
                    break;
                default:
                    ledIO = new LedIO("leds", "LED");
                    break;
            }
            led = new LED(ledIO);
        }
    }

    private void initCommands() {
        if (Constants.swerveEnabled) {
            swerveCommands = new SwerveCommands(swerve, swerveTranslation, swerveRotation);
        }
        if (Constants.hangEnabled) {
            hangCommands = new HangCommands(hang);
        }
        if (Constants.intakeEnabled) {
            intakeCommands = new IntakeCommands(intake);
        }
        if (Constants.shooterEnabled) {
            shooterCommands = new ShooterCommands(shooter);
        }
        if (Constants.ledsEnabled) {
            ledCommands = new LEDCommands(led);
        }
        if (Constants.shooterEnabled) {
            multiCommands = new MultiCommands(shooterCommands, ledCommands, shooter);
        }
    }

    private void configureBindings() {
        /* ---- Main controller bindings ---- */
        /*
         * Reset gyro: create
         * Left stick: drive
         * Right stick X: turn
         * Touchpad: cancel all commands
         */
        testEnabled = new LoggedNetworkBoolean("SmartDashboard/Test/Enabled", false);

        driveController
                .touchpad()
                .or(operator.touchpad())
                .or(otherController.touchpad())
                .onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

        if (Constants.swerveEnabled) {
            driveController.rightMenu().or(otherController.rightMenu()).onTrue(swerveCommands.resetGyro());
            driveController.leftMenu().or(otherController.rightMenu()).onTrue(swerveCommands.lock());
            // Translation: left stick controls dx/dy
            new Trigger(() -> Math.hypot(driveController.getLeftX(), driveController.getLeftY())
                            > Swerve.Constants.moveDeadband)
                    .onTrue(swerveCommands.drive(
                            () -> -driveController.getLeftY(),
                            () -> -driveController.getLeftX(),
                            () -> Swerve.Constants.swerveFieldCentric.get()));

            // Rotation: right stick X controls omega
            new Trigger(() -> Math.abs(driveController.getRightX()) > Swerve.Constants.turnDeadband)
                    .onTrue(swerveCommands.steer(() -> -driveController.getRightX()));

            // Same controls for operator
            new Trigger(() -> Math.hypot(operator.getLeftX(), operator.getLeftY())
                            > Swerve.Constants.moveDeadband)
                    .onTrue(swerveCommands.drive(
                            () -> -operator.getLeftY(),
                            () -> -operator.getLeftX(),
                            () -> Swerve.Constants.swerveFieldCentric.get()));

            new Trigger(() -> Math.abs(operator.getRightX()) > Swerve.Constants.turnDeadband)
                    .onTrue(swerveCommands.steer(() -> -operator.getRightX()));

            // Same controls for other controller
            new Trigger(() -> Math.hypot(otherController.getLeftX(), otherController.getLeftY())
                            > Swerve.Constants.moveDeadband)
                    .onTrue(swerveCommands.drive(
                            () -> -otherController.getLeftY(),
                            () -> -otherController.getLeftX(),
                            () -> Swerve.Constants.swerveFieldCentric.get()));

            new Trigger(() -> Math.abs(otherController.getRightX()) > Swerve.Constants.turnDeadband)
                    .onTrue(swerveCommands.steer(() -> -otherController.getRightX()));

            // Aim at hub: leftBumper on drive, east on other
            otherController
                    .east()
                    .and(() -> !testEnabled.get())
                    .onTrue(swerveCommands.aimAt(Swerve.Constants.hubPosition));
            driveController
                    .leftBumper()
                    .and(() -> !testEnabled.get())
                    .onTrue(swerveCommands.aimAt(Swerve.Constants.hubPosition));

            if (Constants.autoAlignEnabled) {
                // Go to outpost: leftTrigger on drive, south on other
                otherController
                        .south()
                        .and(() -> !testEnabled.get())
                        .onTrue(swerveCommands.setPoseTarget(Swerve.Constants.outpostPosition));
                driveController
                        .leftTrigger()
                        .and(() -> !testEnabled.get())
                        .onTrue(swerveCommands.setPoseTarget(Swerve.Constants.outpostPosition));

                // Go to hang: rightTrigger on drive, west on other
                otherController
                        .west()
                        .and(() -> !testEnabled.get())
                        .onTrue(swerveCommands.setPoseTarget(Swerve.Constants.hangPosition));
                driveController
                        .rightTrigger()
                        .and(() -> !testEnabled.get())
                        .onTrue(swerveCommands.setPoseTarget(Swerve.Constants.hangPosition));
            }
        }
        if (Constants.intakeEnabled) {
            otherController.leftBumper().and(() -> !testEnabled.get()).onTrue(intakeCommands.switchHinge());
            operator.leftBumper().and(() -> !testEnabled.get()).onTrue(intakeCommands.switchHinge());

            otherController.leftTrigger().and(() -> !testEnabled.get()).whileTrue(intakeCommands.intake());
            operator.leftTrigger().and(() -> !testEnabled.get()).whileTrue(intakeCommands.intake());

            otherController.rightBumper().and(() -> !testEnabled.get()).whileTrue(intakeCommands.outtake());
            operator.rightBumper().and(() -> !testEnabled.get()).whileTrue(intakeCommands.outtake());
        }
        if (Constants.shooterEnabled) {
            operator.povLeft().whileTrue(shooterCommands.feedForward());
            operator.povRight().whileTrue(shooterCommands.feedReverse());
        }
        if (multiCommands != null) {
            otherController.rightTrigger().and(() -> !testEnabled.get()).whileTrue(multiCommands.shoot());
            operator.rightTrigger().and(() -> !testEnabled.get()).whileTrue(multiCommands.shoot());
        }
    }

    private void configureTestBindings() {
        /* ---- Test controller bindings ---- */
        /*
         * Forward manual/PID: a
         * Backward manual/PID: b
         */

        testType = new LoggedDashboardChooser<>("Test/Type");
        testType.addDefaultOption("Manual", "Manual");
        testType.addOption("PID", "PID");
        testType.addOption("PIDChange", "PIDChange");

        testSubsystem = new LoggedDashboardChooser<>("Test/Subsystem");
        testSubsystem.addDefaultOption("", ""); // Add default option so code doesn't crash on read

        testSpeed = new LoggedNetworkNumber("SmartDashboard/Test/Speed", 0.2);

        if (Constants.swerveEnabled) {
            testSubsystem.addOption("Swerve", "Swerve");

            // Manual duty cycle forward test
            driveController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.setSpeed(() -> testSpeed.get(), () -> 0, () -> 0))
                    .onFalse(swerveCommands.stop());

            // Manual duty cycle backward test
            driveController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.setSpeed(() -> -testSpeed.get(), () -> 0, () -> 0))
                    .onFalse(swerveCommands.stop());

            // Manual pose reset
            driveController
                    .north()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.resetPose(new Pose2d()));

            // PID to (1,1)
            driveController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PID"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.setPoseTarget(new FieldPose2d(1, 1, new Rotation2d())));

            // PID to (0,0)
            driveController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PID"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.setPoseTarget(new FieldPose2d()));
        }

        if (Constants.hangEnabled) {
            testSubsystem.addOption("Hang", "Hang");
            // Hang move up test
            driveController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Hang"))
                    .whileTrue(hangCommands.setSpeed(() -> testSpeed.get()));

            // Hang move down test
            driveController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Hang"))
                    .whileTrue(hangCommands.setSpeed(() -> -testSpeed.get()));
        }

        if (Constants.shooterEnabled) {
            testSubsystem.addOption("ShooterFeed", "ShooterFeed");
            testSubsystem.addOption("ShooterFly", "ShooterFly");

            // Flywheel forward test
            driveController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("ShooterFly"))
                    .whileTrue(shooterCommands.setFlySpeed(() -> testSpeed.get() * 600));

            // Feed forward test
            driveController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("ShooterFeed"))
                    .whileTrue(shooterCommands.setFeedSpeed(() -> testSpeed.get()));

            // Feed reverse test
            driveController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("ShooterFeed"))
                    .whileTrue(shooterCommands.setFeedSpeed(() -> -testSpeed.get()));
        }

        if (Constants.intakeEnabled) {
            testSubsystem.addOption("Intake", "Intake");
            testSubsystem.addOption("IntakeHinge", "IntakeHinge");

            driveController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Intake"))
                    .whileTrue(intakeCommands.setIntakeSpeed(() -> testSpeed.get()));

            driveController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Intake"))
                    .whileTrue(intakeCommands.setIntakeSpeed(() -> -testSpeed.get()));

            driveController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.setHingeSpeed(() -> testSpeed.get()));

            driveController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.setHingeSpeed(() -> -testSpeed.get()));

            driveController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PID"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .onTrue(intakeCommands.hingeUp());

            driveController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PID"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .onTrue(intakeCommands.hingeDown());

            driveController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PIDChange"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.changeGoal(() -> testSpeed.get() / 10));

            driveController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PIDChange"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.changeGoal(() -> -testSpeed.get() / 10));
        }
    }

    // Refresh drive and operator disconnect alerts
    public void refreshControllerAlerts() {
        controllerDisconnected.set(
                !driveController.isConnected() && Constants.currentMode != Mode.SIM && !otherController.isConnected());
        operatorDisconnected.set(
                !operator.isConnected() && Constants.currentMode != Mode.SIM && !otherController.isConnected());
    }

    // Initialize dashboard auto chooser
    public void configureAuto() {
        // Register named commands for PathPlanner
        if (Constants.intakeEnabled) {
            NamedCommands.registerCommand("IntakeDown", intakeCommands.hingeDown());
            NamedCommands.registerCommand("IntakeUp", intakeCommands.hingeUp());
            NamedCommands.registerCommand("IntakeStart", RobotUtils.schedule(intakeCommands.intake()));
            NamedCommands.registerCommand("IntakeStop", RobotUtils.schedule(intakeCommands.setIntakeSpeed(() -> 0)));
        }

        if (multiCommands != null) {
            NamedCommands.registerCommand("Shoot", RobotUtils.schedule(multiCommands.shoot()));
            NamedCommands.registerCommand("StopShoot", RobotUtils.schedule(multiCommands.shootStop()));
        }

        if (Constants.hangEnabled) {
            NamedCommands.registerCommand("HangUp", RobotUtils.schedule(hangCommands.setSpeed(() -> 0.2)));
            NamedCommands.registerCommand("HangDown", RobotUtils.schedule(hangCommands.setSpeed(() -> -0.2)));
        }

        RobotConfig config;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            Alerts.create("Failed to load robot config!", AlertType.kError);
            e.printStackTrace();
            return;
        }
        if (Constants.swerveEnabled) {
            AutoBuilder.configure(
                    swerve::getPose,
                    swerve::resetPose,
                    swerve::getChassisSpeeds,
                    swerve::setChassisSpeeds,
                    new PPHolonomicDriveController(
                            new PIDConstants(
                                    Swerve.Constants.translationKP.get(),
                                    Swerve.Constants.translationKI.get(),
                                    Swerve.Constants.translationKD.get()),
                            new PIDConstants(
                                    Swerve.Constants.rotationKP.get(),
                                    Swerve.Constants.rotationKI.get(),
                                    Swerve.Constants.rotationKD.get())),
                    config,
                    RobotUtils::onRedAlliance,
                    swerve);

            autoChooser = new LoggedDashboardChooser<Command>("AutoChooser", AutoBuilder.buildAutoChooser("B M"));
        }
    }

    public Command getAutonomousCommand() {
        if (autoChooser != null) {
            return autoChooser.get();
        } else {
            return Commands.none();
        }
    }

    public void periodic() {
        driveController.detectType();
        operator.detectType();

        if (Constants.swerveEnabled) {
            publisher.publish(); // Publish 3D robot data
        }
        refreshControllerAlerts(); // Enable alerts for controller disconnects
    }
}
