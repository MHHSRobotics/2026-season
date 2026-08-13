package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;
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
import frc.robot.util.Field;
import frc.robot.util.FieldPose2d;
import frc.robot.util.RobotUtils;

public class RobotContainer {
    // Subsystems
    private Swerve swerve;
    private SwerveTranslation swerveTranslation;
    private SwerveRotation swerveRotation;
    private Hang hang; // Currently the hang subsystem is not being use don the bot
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

    private final GameController testController = new GameController(2, "Test");

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
        // Initialize swerve
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
            // Translation and rotation are separate subsystems to manage commands' control of movement and rotation separately
            swerveTranslation = new SwerveTranslation();
            swerveRotation = new SwerveRotation();

            if (Constants.visionEnabled) {
                // Create camera variables
                CameraIO frontCam, rightCam, leftCam, backCam;
                switch (Constants.currentMode) {
                    case REAL:
                    case SIM:
                        // If in real bot or sim, use CameraIOPhotonCamera
                        frontCam = new CameraIOPhotonCamera(
                                "FrontCam", "Vision/FrontCam", Swerve.VisionConstants.frontCamPose, 60);
                        rightCam = new CameraIOPhotonCamera(
                                "RightCam", "Vision/RightCam", Swerve.VisionConstants.rightCamPose, 60);
                        leftCam = new CameraIOPhotonCamera(
                                "LeftCam", "Vision/LeftCam", Swerve.VisionConstants.leftCamPose, 60);
                        backCam = new CameraIOPhotonCamera(
                                "BackCam", "Vision/BackCam", Swerve.VisionConstants.backCamPose, 60);
                        break;
                    default:
                        // If in replay use an empty CameraIO
                        frontCam = new CameraIO("FrontCam", "Vision/FrontCam");
                        rightCam = new CameraIO("RightCam", "Vision/RightCam");
                        leftCam = new CameraIO("LeftCam", "Vision/LeftCam");
                        backCam = new CameraIO("BackCam", "Vision/BackCam");
                        break;
                }
                // Add cameras to swerve ododmetry
                swerve.addCameraSource(frontCam);
                swerve.addCameraSource(rightCam);
                swerve.addCameraSource(leftCam);
                swerve.addCameraSource(backCam);
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
        // Initialize shooter
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
        // Initialize hang
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
            hang = new Hang(hangMotor, hangEncoder);
        }
        // Initialize intake
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
        // Initialize LEDs
        if (Constants.ledsEnabled) {
            LedIO backCandle;
            LedIO frontCandle;
            switch (Constants.currentMode) {
                case REAL:
                case SIM:
                    backCandle = new LedIOCANdle("back candle", "LED/BackCandle", LED.Constants.backId);
                    frontCandle = new LedIOCANdle("front candle", "LED/FrontCandle", LED.Constants.frontId);
                    break;
                default:
                    backCandle = new LedIO("back candle", "LED/BackCandle");
                    frontCandle = new LedIO("front candle", "LED/FrontCandle");

                    break;
            }
            led = new LED(frontCandle, backCandle, shooter, swerve);
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
        multiCommands = new MultiCommands(shooterCommands, intakeCommands, swerveCommands, swerve);
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
                .or(testController.touchpad())
                .onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

        if (Constants.swerveEnabled) {
            driveController
                    .rightMenu()
                    .or(testController.rightMenu())
                    .or(operator.rightMenu())
                    .onTrue(swerveCommands.resetGyro());
            driveController
                    .rightTrigger()
                    .or(testController.leftMenu())
                    .or(operator.leftMenu())
                    .onTrue(swerveCommands.lock());
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

            // Same controls for operator when FMS isn't attached
            if (!DriverStation.isFMSAttached()) {
                new Trigger(() -> Math.hypot(operator.getLeftX(), operator.getLeftY()) > Swerve.Constants.moveDeadband)
                        .onTrue(swerveCommands.drive(
                                () -> -operator.getLeftY(),
                                () -> -operator.getLeftX(),
                                () -> Swerve.Constants.swerveFieldCentric.get()));
                
                new Trigger(() -> Math.abs(operator.getRightX()) > Swerve.Constants.turnDeadband)
                    .onTrue(swerveCommands.steer(() -> -operator.getRightX()));
            }

            // Same controls for test controller
            new Trigger(() -> Math.hypot(testController.getLeftX(), testController.getLeftY())
                            > Swerve.Constants.moveDeadband)
                    .onTrue(swerveCommands.drive(
                            () -> -testController.getLeftY(),
                            () -> -testController.getLeftX(),
                            () -> Swerve.Constants.swerveFieldCentric.get()));

            new Trigger(() -> Math.abs(testController.getRightX()) > Swerve.Constants.turnDeadband)
                    .onTrue(swerveCommands.steer(() -> -testController.getRightX()));

            // Aim at hub: leftBumper on drive, east on test
            testController.east().and(() -> !testEnabled.get()).onTrue(multiCommands.aimAtHub());
            driveController.leftBumper().onTrue(multiCommands.aimAtHub());

            if (Constants.autoAlignEnabled) {
                // Go to outpost: leftTrigger on drive, south on other
                testController
                        .south()
                        .and(() -> !testEnabled.get())
                        .onTrue(swerveCommands.setPoseTarget(Field.outpostPosition));
                driveController.leftTrigger().onTrue(swerveCommands.setPoseTarget(Field.outpostPosition));

                // Go to hang: rightTrigger on drive, west on other
                testController
                        .west()
                        .and(() -> !testEnabled.get())
                        .onTrue(swerveCommands.setPoseTarget(Field.hangPosition));
                driveController.rightTrigger().onTrue(swerveCommands.setPoseTarget(Field.hangPosition));
            }
        }
        if (Constants.intakeEnabled) {
            testController.leftBumper().and(() -> !testEnabled.get()).onTrue(intakeCommands.switchHinge());
            operator.leftBumper().whileTrue(intakeCommands.switchHinge());

            testController.north().and(() -> !testEnabled.get()).whileTrue(intakeCommands.setHingeUpShort());
            operator.east().whileTrue(intakeCommands.setHingeUpShort());

            testController.leftTrigger().and(() -> !testEnabled.get()).whileTrue(multiCommands.intakeWithSpeed());
            operator.leftTrigger().whileTrue(multiCommands.intakeWithSpeed());

            testController.rightBumper().and(() -> !testEnabled.get()).whileTrue(intakeCommands.outtake());
            operator.rightBumper().whileTrue(intakeCommands.outtake());
        }
        if (Constants.shooterEnabled) {
            operator.povLeft().whileTrue(shooterCommands.feedForward());
            operator.povRight().whileTrue(shooterCommands.feedReverse());

            testController.rightBumper().and(() -> !testEnabled.get()).whileTrue(shooterCommands.feedReverse());
            operator.rightBumper().whileTrue(shooterCommands.feedReverse());

            testController.rightTrigger().and(() -> !testEnabled.get()).whileTrue(multiCommands.shoot());
            testController.south().and(() -> !testEnabled.get()).whileTrue(multiCommands.shootDefault());
            operator.rightTrigger().whileTrue(multiCommands.shoot());
            operator.south().whileTrue(multiCommands.shootDefault());
        }
        if (Constants.hangEnabled) {
            operator.povUp().whileTrue(hangCommands.setSpeed(() -> 0.2));
            operator.povDown().whileTrue(hangCommands.setSpeed(() -> -0.2));
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
            testController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.setSpeed(() -> testSpeed.get(), () -> 0, () -> 0))
                    .onFalse(swerveCommands.stop());

            // Manual duty cycle backward test
            testController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.setSpeed(() -> -testSpeed.get(), () -> 0, () -> 0))
                    .onFalse(swerveCommands.stop());

            // Manual pose reset
            testController
                    .north()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.resetPose(new Pose2d()));

            // PID to (1,1)
            testController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PID"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.setPoseTarget(new FieldPose2d(1, 1, new Rotation2d())));

            // PID to (0,0)
            testController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PID"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.setPoseTarget(new FieldPose2d()));
        }

        if (Constants.hangEnabled) {
            testSubsystem.addOption("Hang", "Hang");
            // Hang move up test
            testController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Hang"))
                    .whileTrue(hangCommands.setSpeed(() -> testSpeed.get()));

            // Hang move down test
            testController
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
            testController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("ShooterFly"))
                    .whileTrue(shooterCommands.setFlySpeed(() -> testSpeed.get()));

            // Flywheel shoot
            testController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("ShooterFly"))
                    .whileTrue(shooterCommands.shoot(() -> testSpeed.get()));

            // Feed forward test
            testController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("ShooterFeed"))
                    .whileTrue(shooterCommands.setFeedSpeed(() -> testSpeed.get()));

            // Feed reverse test
            testController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("ShooterFeed"))
                    .whileTrue(shooterCommands.setFeedSpeed(() -> -testSpeed.get()));
        }

        if (Constants.intakeEnabled) {
            testSubsystem.addOption("Intake", "Intake");
            testSubsystem.addOption("IntakeHinge", "IntakeHinge");

            testController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Intake"))
                    .whileTrue(intakeCommands.setRollerSpeed(() -> testSpeed.get()));

            testController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Intake"))
                    .whileTrue(intakeCommands.setRollerSpeed(() -> -testSpeed.get()));

            testController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PID"))
                    .and(() -> testSubsystem.get().equals("Intake"))
                    .whileTrue(intakeCommands.setRollerTargetSpeed(() -> testSpeed.get()));

            testController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PID"))
                    .and(() -> testSubsystem.get().equals("Intake"))
                    .whileTrue(intakeCommands.setRollerTargetSpeed(() -> -testSpeed.get()));

            testController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.setHingeSpeed(() -> testSpeed.get()));

            testController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.setHingeSpeed(() -> -testSpeed.get()));

            testController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PID"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .onTrue(intakeCommands.setHingeUp());

            testController
                    .east()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PID"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .onTrue(intakeCommands.setHingeDown());

            testController
                    .south()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PIDChange"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.changeGoal(() -> testSpeed.get() / 10));

            testController
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
                !driveController.isConnected() && Constants.currentMode != Mode.SIM && !testController.isConnected());
        operatorDisconnected.set(
                !operator.isConnected() && Constants.currentMode != Mode.SIM && !testController.isConnected());
    }

    // Initialize dashboard auto chooser
    public void configureAuto() {
        autoChooser = new LoggedDashboardChooser<Command>("AutoChooser");
        autoChooser.addDefaultOption("None", Commands.none());
        // 1 shot autos
        autoChooser.addOption("LI_LS", multiCommands.getSingleAuto("LI_LS", false));
        autoChooser.addOption("RI_RS", multiCommands.getSingleAuto("LI_LS", true));
        // autoChooser.addOption("LI_LD", multiCommands.getSingleAuto("LI_LD", false));
        // autoChooser.addOption("RI_RD", multiCommands.getSingleAuto("RI_RD", false));
        autoChooser.addOption("LI_LS_N", multiCommands.getSingleAuto("LI_LS_N", false));
        autoChooser.addOption("RI_RS_N", multiCommands.getSingleAuto("LI_LS_N", true));
        // autoChooser.addOption("LI_RS_N", multiCommands.getSingleAuto("LI_RS_N", false));
        // autoChooser.addOption("RI_LS_N", multiCommands.getSingleAuto("LI_RS_N", true));

        // 2 shot autos
        // autoChooser.addOption("LI_LS|LS_RS_N", multiCommands.getDoubleAuto("LI_LS", false, "LS_RS_N", false));
        autoChooser.addOption("LI_LS|LS_LS_N", multiCommands.getDoubleAuto("LI_LS", false, "LS_LS_N", false));
        // autoChooser.addOption("RI_RS|RS_LS_N", multiCommands.getDoubleAuto("LI_LS", true, "LS_RS_N", true));
        autoChooser.addOption("RI_RS|RS_RS_N", multiCommands.getDoubleAuto("LI_LS", true, "LS_LS_N", true));
        // autoChooser.addOption("LI_LD|LD_RS_N", multiCommands.getDoubleAuto("LI_LD", false, "LD_RS_N", false));
        // autoChooser.addOption("LI_LD|LD_LS_N", multiCommands.getDoubleAuto("LI_LD", false, "LD_LS_N", false));
        // autoChooser.addOption("RI_RD|LD_RS_N", multiCommands.getDoubleAuto("RI_RD", false, "RD_LS_N", false));
        // autoChooser.addOption("RI_RD|LD_RS_N", multiCommands.getDoubleAuto("RI_RD", false, "RD_RS_N", false));
        // autoChooser.addOption("LI_LS_N|LS_LD", multiCommands.getDoubleAuto("LI_LS_N", false, "LS_LD", false));
        // autoChooser.addOption("LI_RS_N|RS_RD", multiCommands.getDoubleAuto("LI_RS_N", false, "RS_RD", false));
        // autoChooser.addOption("RI_LS_N|LS_LD", multiCommands.getDoubleAuto("LI_RS_N", true, "LS_LD", false));
        // autoChooser.addOption("RI_RS_N|RS_RD", multiCommands.getDoubleAuto("LI_LS_N", true, "RS_RD", false));
        autoChooser.addOption("LI_LS_N|LS_LS_N", multiCommands.getDoubleAuto("LI_LS_N", false, "LS_LS_N", false));
        // autoChooser.addOption("LI_LS_N|LS_RS_N", multiCommands.getDoubleAuto("LI_LS_N", false, "LS_RS_N", false));
        // autoChooser.addOption("LI_RS_N|RS_LS_N", multiCommands.getDoubleAuto("LI_RS_N", false, "LS_RS_N", true));
        // autoChooser.addOption("LI_RS_N|RS_RS_N", multiCommands.getDoubleAuto("LI_RS_N", false, "LS_LS_N", true));
        // autoChooser.addOption("RI_LS_N|LS_LS_N", multiCommands.getDoubleAuto("LI_RS_N", true, "LS_LS_N", false));
        // autoChooser.addOption("RI_LS_N|LS_RS_N", multiCommands.getDoubleAuto("LI_RS_N", true, "LS_RS_N", false));
        // autoChooser.addOption("RI_RS_N|RS_LS_N", multiCommands.getDoubleAuto("LI_LS_N", true, "LS_RS_N", true));
        autoChooser.addOption("RI_RS_N|RS_RS_N", multiCommands.getDoubleAuto("LI_LS_N", true, "LS_LS_N", true));

        autoChooser.addOption("RB_RBS", multiCommands.getSingleAuto("LB_LBS", true));
        autoChooser.addOption("LB_LBS", multiCommands.getSingleAuto("LB_LBS", false));
        autoChooser.addOption("C_CS", multiCommands.getSingleAuto("C_CS", true));

        if (Constants.swerveEnabled) {
            // Register named commands for PathPlanner
            if (Constants.intakeEnabled) {
                NamedCommands.registerCommand("IntakeDown", intakeCommands.setHingeDown());
                NamedCommands.registerCommand("IntakeUp", intakeCommands.setHingeUp());
                NamedCommands.registerCommand("IntakeStart", RobotUtils.schedule(intakeCommands.intake()));
                NamedCommands.registerCommand("IntakeStop", RobotUtils.schedule(intakeCommands.rollerStop()));
            }

            if (Constants.shooterEnabled) {
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
            for (String auto : AutoBuilder.getAllAutoNames()) {
                autoChooser.addOption("PP_" + auto, AutoBuilder.buildAuto(auto));
            }
        }
    }

    public Command getAutonomousCommand() {
        if(autoChooser==null){
            return Commands.none();
        }
        return autoChooser.get();
    }

    // Flashes a color shortly before each shift change and end game so the driver gets a heads up in Elastic
    public void refreshFlashSignal() {
        double matchTime = DriverStation.getMatchTime();
        double activeWarningTime = Double.POSITIVE_INFINITY;
        String activeColor = null;

        // The warning times and their colors are parallel arrays, so ignore any entry missing its counterpart
        int warningCount = Math.min(Constants.shiftFlashWarningTimes.length, Constants.shiftFlashColors.length);

        // Only warn during teleop, and only when the driver station is actually reporting a match clock
        if (DriverStation.isTeleop() && matchTime > 0) {
            for (double shiftTime : Constants.shiftChangeTimes) {
                double timeUntilShift = matchTime - shiftTime;
                for (int i = 0; i < warningCount; i++) {
                    double warningTime = Constants.shiftFlashWarningTimes[i];
                    // Skip warnings that would land at or before the start of teleop. The transition shift is only
                    // 10s long, so its 10s warning would otherwise fire the instant teleop begins.
                    if (shiftTime + warningTime >= Constants.teleopDuration) {
                        continue;
                    }
                    if (timeUntilShift <= warningTime
                            && timeUntilShift > warningTime - Constants.shiftFlashDuration
                            && warningTime < activeWarningTime) {
                        // Prefer the most urgent warning if two ever overlap
                        activeWarningTime = warningTime;
                        activeColor = Constants.shiftFlashColors[i];
                    }
                }
            }
        }

        // Blink while a warning is active so the indicator reads as a flash instead of a steady light
        boolean blinkOn = (int) Math.floor(Timer.getFPGATimestamp() * Constants.shiftFlashBlinkRate * 2) % 2 == 0;

        Logger.recordOutput(
                "ShiftChangeFlash", activeColor != null && blinkOn ? activeColor : Constants.shiftFlashOffColor);
    }

    public void periodic() {
        driveController.detectType();
        operator.detectType();
        testController.detectType();

        if (Constants.swerveEnabled) {
            publisher.publish(); // Publish 3D robot data
        }
        refreshControllerAlerts(); // Enable alerts for controller disconnects
        refreshFlashSignal(); // Enable flash signal in Elastic
    }
}
