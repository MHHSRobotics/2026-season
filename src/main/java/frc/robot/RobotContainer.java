package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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
import frc.robot.commands.HopperCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.LEDCommands;
import frc.robot.commands.MultiCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.io.CameraIO;
import frc.robot.io.CameraIOPhotonCamera;
import frc.robot.io.EncoderIO;
import frc.robot.io.EncoderIOCANcoder;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon;
import frc.robot.io.LedIO;
import frc.robot.io.LedIOCANdle;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIOTalonFX;
import frc.robot.network.RobotPublisher;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperPhysicsSim;
import frc.robot.subsystems.hopper.HopperSim;
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
import frc.robot.util.FieldPose2d;
import frc.robot.util.RobotUtils;

public class RobotContainer {
    // Subsystems
    private Swerve swerve;
    private SwerveTranslation swerveTranslation;
    private SwerveRotation swerveRotation;
    private Hang hang;
    private Hopper hopper;
    private Intake intake;
    private Shooter shooter;
    private LED led;

    private SwerveCommands swerveCommands;
    private HangCommands hangCommands;
    private HopperCommands hopperCommands;
    private IntakeCommands intakeCommands;
    private ShooterCommands shooterCommands;
    private LEDCommands ledCommands;

    private MultiCommands multiCommands;

    private final CommandPS5Controller driveController = new CommandPS5Controller(0);

    private final CommandPS5Controller operator =
            new CommandPS5Controller(1); // Manual controller for subsystems, for continuous change in PID goal

    private LoggedNetworkBoolean testEnabled;
    private LoggedNetworkBoolean altControls;
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
                CameraIO hubCam, hubLeftCam, hubRightCam, hangCam;
                switch (Constants.currentMode) {
                    case REAL:
                    case SIM:
                        // If in real bot or sim, use CameraIOPhotonCamera
                        hubCam = new CameraIOPhotonCamera(
                                "hub camera", "Vision/HubCam", Swerve.VisionConstants.hubCamPose, 60);
                        hubLeftCam = new CameraIOPhotonCamera(
                                "left hub camera", "Vision/LeftHubCam", Swerve.VisionConstants.hubLeftCamPose, 60);
                        hubRightCam = new CameraIOPhotonCamera(
                                "right hub camera", "Vision/RightHubCam", Swerve.VisionConstants.hubRightCamPose, 60);
                        hangCam = new CameraIOPhotonCamera(
                                "hang camera", "Vision/HangCam", Swerve.VisionConstants.hangCamPose, 60);
                        break;
                    default:
                        // If in replay use an empty CameraIO
                        hubCam = new CameraIO("hub camera", "Vision/HubCam");
                        hubLeftCam = new CameraIO("left hub camera", "Vision/LeftHubCam");
                        hubRightCam = new CameraIO("right hub camera", "Vision/RightHubCam");
                        hangCam = new CameraIO("hang camera", "Vision/HangCam");
                        break;
                }
                // Add cameras to swerve ododmetry
                swerve.addCameraSource(hubCam);
                swerve.addCameraSource(hubLeftCam);
                swerve.addCameraSource(hubRightCam);
                swerve.addCameraSource(hangCam);
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
            MotorIO feedMotor, flyMotor;
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
                    break;
                default:
                    feedMotor = new MotorIO("shooter feed motor", "Shooter/Feed");
                    flyMotor = new MotorIO("shooter fly motor", "Shooter/Flywheel");
                    break;
            }
            shooter = new Shooter(feedMotor, flyMotor);

            if (Constants.currentMode == Mode.SIM) {
                if (!Constants.enablePhysicsSim) {
                    new ShooterSim(feedMotor, flyMotor);
                } else {
                    new ShooterPhysicsSim(feedMotor, flyMotor, "/MuJoCo/Shooter");
                }
            }
        }

        if (Constants.hopperEnabled) {
            MotorIO hopperMotor;
            switch (Constants.currentMode) {
                case REAL:
                case SIM:
                    hopperMotor = new MotorIOTalonFX(
                            Hopper.Constants.motorId, Constants.defaultBus, "hopper motor", "Hopper/Motor");
                    break;
                default:
                    hopperMotor = new MotorIO("hopper motor", "Hopper/Motor");
                    break;
            }
            hopper = new Hopper(hopperMotor);

            if (Constants.currentMode == Mode.SIM) {
                if (!Constants.enablePhysicsSim) {
                    new HopperSim(hopperMotor);
                } else {
                    new HopperPhysicsSim(hopperMotor, "/MuJoCo/Hopper");
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
                    break;
                default:
                    rollerMotor = new MotorIO("intake roller motor", "Intake/Roller");
                    hingeMotor = new MotorIO("intake hinge motor", "Intake/Hinge");
                    break;
            }
            intake = new Intake(rollerMotor, hingeMotor);

            if (Constants.currentMode == Mode.SIM) {
                if (!Constants.physicsSimEnabled) {
                    new IntakeSim(rollerMotor, hingeMotor);
                } else {
                    new IntakePhysicsSim(rollerMotor, hingeMotor, "/MuJoCo/Intake");
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
        if (Constants.hopperEnabled) {
            hopperCommands = new HopperCommands(hopper);
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
        if (Constants.intakeEnabled && Constants.shooterEnabled && Constants.hopperEnabled) {
            multiCommands = new MultiCommands(hopperCommands, intakeCommands, shooterCommands, ledCommands, shooter);
        }
    }

    private boolean altControls() {
        return altControls.get() ? true : DriverStation.isFMSAttached();
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
        altControls = new LoggedNetworkBoolean("AltControlsEnabled", false);

        if (Constants.swerveEnabled) {
            driveController.options().onTrue(swerveCommands.resetGyro());
            driveController.create().onTrue(swerveCommands.lock());
            // Translation: left stick controls dx/dy
            new Trigger(() -> Math.hypot(driveController.getLeftX(), driveController.getLeftY())
                            > Swerve.Constants.moveDeadband)
                    .onTrue(swerveCommands.drive(
                            () -> -driveController.getLeftY(),
                            () -> -driveController.getLeftX(),
                            () -> Swerve.Constants.swerveFieldCentric.get()));

            // Rotation: right stick X controls omega
            driveController
                    .axisMagnitudeGreaterThan(2, Swerve.Constants.turnDeadband)
                    .onTrue(swerveCommands.steer(() -> -driveController.getRightX()));

            // Aim at hub: circle, L1 in alt controls
            driveController
                    .circle()
                    .and(() -> !altControls())
                    .onTrue(swerveCommands.aimAt(Swerve.Constants.hubPosition));
            driveController.L1().and(() -> altControls()).onTrue(swerveCommands.aimAt(Swerve.Constants.hubPosition));

            if (Constants.autoAlignEnabled) {
                // Go to outpost: cross, L2 in alt controls
                driveController
                        .cross()
                        .and(() -> !altControls())
                        .onTrue(swerveCommands.setPoseTarget(Swerve.Constants.outpostPosition));
                driveController
                        .L2()
                        .and(() -> altControls())
                        .onTrue(swerveCommands.setPoseTarget(Swerve.Constants.outpostPosition));

                // Go to hang: cross, L2 in alt controls
                driveController
                        .square()
                        .and(() -> !altControls())
                        .onTrue(swerveCommands.setPoseTarget(Swerve.Constants.hangPosition));
                driveController
                        .R2()
                        .and(() -> altControls())
                        .onTrue(swerveCommands.setPoseTarget(Swerve.Constants.hangPosition));
            }

            driveController.touchpad().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance()
                    .cancelAll()));
        }
        if (Constants.intakeEnabled) {
            // Toggle hinge is driver L1 on main controls, operator L1 on alt
            driveController
                    .L1()
                    .and(() -> !testEnabled.get())
                    .and(() -> !altControls())
                    .onTrue(intakeCommands.switchHinge());
            operator.L1().and(() -> !testEnabled.get()).and(() -> altControls()).onTrue(intakeCommands.switchHinge());

            // Intake is driver L2 on main controls, operator L2 on alt
            driveController
                    .L2()
                    .and(() -> !testEnabled.get())
                    .and(() -> !altControls())
                    .whileTrue(intakeCommands.intake());
            operator.L2().and(() -> !testEnabled.get()).and(() -> altControls()).onTrue(intakeCommands.intake());

            // Outtake is driver R1 on main controls, operator R1 on alt
            driveController
                    .R1()
                    .and(() -> !testEnabled.get())
                    .and(() -> !altControls())
                    .whileTrue(intakeCommands.outtake());
            operator.R1().and(() -> !testEnabled.get()).and(() -> altControls()).onTrue(intakeCommands.outtake());
        }
        if (Constants.hopperEnabled) {
            // Hopper in is driver povUp on main controls, operator povUp on alt
            driveController
                    .povUp()
                    .and(() -> !testEnabled.get())
                    .and(() -> !altControls())
                    .whileTrue(hopperCommands.forward());
            operator.povUp()
                    .and(() -> !testEnabled.get())
                    .and(() -> altControls())
                    .whileTrue(hopperCommands.forward());

            // Hopper out is driver povUp on main controls, operator povUp on alt
            driveController
                    .povDown()
                    .and(() -> !testEnabled.get())
                    .and(() -> !altControls())
                    .whileTrue(hopperCommands.reverse());
            operator.povDown()
                    .and(() -> !testEnabled.get())
                    .and(() -> altControls())
                    .whileTrue(hopperCommands.reverse());
        }
        if (multiCommands != null) {
            // Shoot is driver R2 on main controls, operator R2 on alt
            driveController
                    .R2()
                    .and(() -> !testEnabled.get())
                    .and(() -> !altControls())
                    .whileTrue(multiCommands.shoot());
            operator.R2().and(() -> !testEnabled.get()).and(() -> altControls()).whileTrue(multiCommands.shoot());
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
                    .cross()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.setSpeed(testSpeed.get(), 0, 0))
                    .onFalse(swerveCommands.stop());

            // Manual duty cycle backward test
            driveController
                    .circle()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.setSpeed(-testSpeed.get(), 0, 0))
                    .onFalse(swerveCommands.stop());

            // Manual pose reset
            driveController
                    .triangle()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.resetPose(new Pose2d()));

            // PID to (1,1)
            driveController
                    .cross()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PID"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.setPoseTarget(new FieldPose2d(1, 1, new Rotation2d())));

            // PID to (0,0)
            driveController
                    .circle()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PID"))
                    .and(() -> testSubsystem.get().equals("Swerve"))
                    .onTrue(swerveCommands.setPoseTarget(new FieldPose2d()));
        }

        if (Constants.hangEnabled) {
            testSubsystem.addOption("Hang", "Hang");
            // Hang move up test
            driveController
                    .cross()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Hang"))
                    .whileTrue(hangCommands.setSpeed(() -> testSpeed.get()));

            // Hang move down test
            driveController
                    .circle()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Hang"))
                    .whileTrue(hangCommands.setSpeed(() -> -testSpeed.get()));
        }

        if (Constants.hopperEnabled) {
            testSubsystem.addOption("Hopper", "Hopper");

            // Hopper slow forward test
            driveController
                    .cross()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Hopper"))
                    .whileTrue(hopperCommands.setSpeed(() -> testSpeed.get()));

            // Hopper slow reverse test
            driveController
                    .circle()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Hopper"))
                    .whileTrue(hopperCommands.setSpeed(() -> -testSpeed.get()));
        }

        if (Constants.shooterEnabled) {
            testSubsystem.addOption("ShooterFeed", "ShooterFeed");
            testSubsystem.addOption("ShooterFly", "ShooterFly");

            // Flywheel forward test
            driveController
                    .cross()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("ShooterFly"))
                    .whileTrue(shooterCommands.setFlySpeed(() -> testSpeed.get() * 600));

            // Feed forward test
            driveController
                    .cross()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("ShooterFeed"))
                    .whileTrue(shooterCommands.setFeedSpeed(() -> testSpeed.get()));

            // Feed reverse test
            driveController
                    .circle()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("ShooterFeed"))
                    .whileTrue(shooterCommands.setFeedSpeed(() -> -testSpeed.get()));
        }

        if (Constants.intakeEnabled) {
            testSubsystem.addOption("Intake", "Intake");
            testSubsystem.addOption("IntakeHinge", "IntakeHinge");

            driveController
                    .cross()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Intake"))
                    .whileTrue(intakeCommands.setIntakeSpeed(() -> testSpeed.get()));

            driveController
                    .circle()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("Intake"))
                    .whileTrue(intakeCommands.setIntakeSpeed(() -> -testSpeed.get()));

            driveController
                    .cross()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.setHingeSpeed(() -> testSpeed.get()));

            driveController
                    .circle()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("Manual"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.setHingeSpeed(() -> -testSpeed.get()));

            driveController
                    .cross()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PID"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .onTrue(intakeCommands.hingeUp());

            driveController
                    .circle()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PID"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .onTrue(intakeCommands.hingeDown());

            driveController
                    .cross()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PIDChange"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.changeGoal(() -> testSpeed.get() / 10));

            driveController
                    .circle()
                    .and(() -> testEnabled.get())
                    .and(() -> testType.get().equals("PIDChange"))
                    .and(() -> testSubsystem.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.changeGoal(() -> -testSpeed.get() / 10));
        }
    }

    // Refresh drive and operator disconnect alerts
    public void refreshControllerAlerts() {
        controllerDisconnected.set(!driveController.isConnected() && Constants.currentMode != Mode.SIM);
        operatorDisconnected.set(!operator.isConnected() && Constants.currentMode != Mode.SIM);
    }

    // Initialize dashboard auto chooser
    public void configureAuto() {
        // Register named commands for PathPlanner
        NamedCommands.registerCommand("IntakeDown", intakeCommands.hingeDown());
        NamedCommands.registerCommand("IntakeUp", intakeCommands.hingeUp());
        NamedCommands.registerCommand("IntakeStart", Commands.runOnce(() -> intake.intake()));
        NamedCommands.registerCommand("IntakeStop", Commands.runOnce(() -> intake.intakeStop()));

        NamedCommands.registerCommand("Feed", Commands.runOnce(() -> shooter.feedShoot()));
        NamedCommands.registerCommand("Shoot", Commands.runOnce(() -> shooter.flyShoot()));
        NamedCommands.registerCommand("StopShoot", Commands.runOnce(() -> shooter.flyStop()));
        NamedCommands.registerCommand("StopFeed", Commands.runOnce(() -> shooter.feedStop()));

        NamedCommands.registerCommand("HopperStart", Commands.runOnce(() -> hopper.forward()));
        NamedCommands.registerCommand("HopperStop", Commands.runOnce(() -> hopper.stop()));
        NamedCommands.registerCommand("HangUp", Commands.runOnce(() -> hang.setSpeed(0.2)));
        NamedCommands.registerCommand("HangDown", Commands.runOnce(() -> hang.setSpeed(-0.2)));

        RobotConfig config;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
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
                                Swerve.Constants.driveKP.get(),
                                Swerve.Constants.driveKI.get(),
                                Swerve.Constants.driveKD.get()),
                        new PIDConstants(
                                Swerve.Constants.rotationKP.get(),
                                Swerve.Constants.rotationKI.get(),
                                Swerve.Constants.rotationKD.get())),
                config,
                RobotUtils::onRedAlliance,
                swerve);

        autoChooser = new LoggedDashboardChooser<Command>("AutoChooser", AutoBuilder.buildAutoChooser("Cat"));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void periodic() {
        if (Constants.swerveEnabled) {
            publisher.publish(); // Publish 3D robot data
        }
        refreshControllerAlerts(); // Enable alerts for controller disconnects
    }
}
