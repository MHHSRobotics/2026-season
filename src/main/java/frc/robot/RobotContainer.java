package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.Constants.Mode;
import frc.robot.commands.HangCommands;
import frc.robot.commands.HopperCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.MultiCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.io.BitIO;
import frc.robot.io.BitIODigitalSignal;
import frc.robot.io.CameraIO;
import frc.robot.io.CameraIOPhotonCamera;
import frc.robot.io.EncoderIO;
import frc.robot.io.EncoderIOCANcoder;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon;
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
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.swerve.VisionSim;
import frc.robot.util.Alerts;

public class RobotContainer {
    // Subsystems
    private Swerve swerve;
    private Hang hang;
    private Hopper hopper;
    private Intake intake;
    private Shooter shooter;

    private SwerveCommands swerveCommands;
    private HangCommands hangCommands;
    private HopperCommands hopperCommands;
    private IntakeCommands intakeCommands;
    private ShooterCommands shooterCommands;

    private MultiCommands multiCommands;

    private final CommandPS5Controller driveController = new CommandPS5Controller(0);

    private final CommandPS5Controller operator =
            new CommandPS5Controller(1); // Manual controller for subsystems, for continuous change in PID goal

    private final CommandPS5Controller testController = new CommandPS5Controller(
            2); // Test controller for controlling one subsystem at a time, for full manual and PID movements

    private LoggedDashboardChooser<String> testControllerChooser; // Which subsystem the test controller is applied to
    private LoggedDashboardChooser<String>
            testControllerManual; // Whether to use manual or PID mode for the test controller

    private LoggedDashboardChooser<String> autoChooser; // Choice of auto

    private RobotPublisher publisher; // Publishes 3D robot data to AdvantageScope for visualization

    // Alerts for disconnected controllers
    private Alert controllerDisconnected = new Alert("Drive controller is disconnected", AlertType.kWarning);
    private Alert operatorDisconnected = new Alert("Operator is disconnected", AlertType.kWarning);

    public RobotContainer() {
        initSubsystems(); // Initialize all the IO objects, subsystems, and mechanism simulators
        initCommands(); // Initialize command classes

        configureBindings(); // Add drive controller bindings

        configureManualBindings(); // Configure bindings for manual controller

        // Configure bindings for test controller when not in match
        if (!DriverStation.isFMSAttached()) {
            configureTestBindings();
        }

        configureAutoChooser(); // Set up the auto chooser

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
            hang = new Hang(hangMotor);
        }

        if (Constants.intakeEnabled) {
            MotorIO intakeMotor;
            MotorIO hingeMotor;
            BitIO leftSwitch;
            BitIO rightSwitch;
            switch (Constants.currentMode) {
                case REAL:
                case SIM:
                    intakeMotor = new MotorIOTalonFX(
                            Intake.Constants.intakeMotorId,
                            Constants.defaultBus,
                            "intake flywheel motor",
                            "Intake/Flywheel");
                    hingeMotor = new MotorIOTalonFX(
                            Intake.Constants.hingeMotorId, Constants.defaultBus, "intake hinge motor", "Intake/Hinge");
                    leftSwitch = new BitIODigitalSignal(
                            "intake left limit switch", "Intake/LeftSwitch", Intake.Constants.leftSwitchId);
                    rightSwitch = new BitIODigitalSignal(
                            "intake right limit switch", "Intake/RightSwitch", Intake.Constants.rightSwitchId);
                    break;
                default:
                    intakeMotor = new MotorIO("intake flywheel motor", "Intake/Flywheel");
                    hingeMotor = new MotorIO("intake hinge motor", "Intake/Hinge");
                    leftSwitch = new BitIO("intake left limit switch", "Intake/LeftSwitch");
                    rightSwitch = new BitIO("intake right limit switch", "Intake/RightSwitch");
                    break;
            }
            intake = new Intake(intakeMotor, hingeMotor, leftSwitch, rightSwitch);

            if (Constants.currentMode == Mode.SIM) {
                if (!Constants.physicsSimEnabled) {
                    new IntakeSim(intakeMotor, hingeMotor);
                } else {
                    new IntakePhysicsSim(intakeMotor, hingeMotor, "/MuJoCo/Intake");
                }
            }
        }
    }

    private void initCommands() {
        if (Constants.swerveEnabled) {
            swerveCommands = new SwerveCommands(swerve);
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
        if (Constants.intakeEnabled && Constants.shooterEnabled && Constants.hopperEnabled) {
            multiCommands = new MultiCommands(hopperCommands, intakeCommands, shooterCommands);
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

        if (Constants.swerveEnabled) {
            driveController.options().onTrue(swerveCommands.resetGyro());
            driveController.create().onTrue(swerveCommands.lock());
            /*
             * How this works:
             * When the driver controller is outside of its deadband, it runs swerveCommands.drive(), which overrides auto align commands. swerveCommands.drive() will continue to run until an auto align command is executed, so the swerve drive will stop when both sticks are at 0.
             */
            driveController
                    .axisMagnitudeGreaterThan(2, Swerve.Constants.turnDeadband)
                    .or(() -> Math.hypot(driveController.getLeftX(), driveController.getLeftY())
                            > Swerve.Constants.moveDeadband)
                    .onTrue(swerveCommands.drive(
                            () -> -driveController.getLeftY(),
                            () -> -driveController.getLeftX(),
                            () -> -driveController.getRightX(),
                            () -> Swerve.Constants.swerveFieldCentric.get()));

            driveController.touchpad().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance()
                    .cancelAll()));
        }
        if (Constants.intakeEnabled) {
            driveController.L1().onTrue(intakeCommands.switchHinge());
            driveController.L2().whileTrue(intakeCommands.intake());
            driveController.R1().whileTrue(intakeCommands.outtake());
        }
        if (Constants.hopperEnabled) {
            driveController.povUp().whileTrue(hopperCommands.forward());
            driveController.povDown().whileTrue(hopperCommands.reverse());
        }
        if (Constants.shooterEnabled) {
            driveController.R2().whileTrue(multiCommands.shoot());
        }
    }

    private void configureTestBindings() {
        /* ---- Test controller bindings ---- */
        /*
         * Forward manual/PID: cross
         * Backward manual/PID: circle
         */

        testControllerManual = new LoggedDashboardChooser<>("Test/Type");
        testControllerManual.addDefaultOption("Manual", "Manual");
        testControllerManual.addOption("Fast", "Fast");
        testControllerManual.addOption("PID", "PID");
        testControllerManual.addOption("PIDChange", "PIDChange");

        testControllerChooser = new LoggedDashboardChooser<>("Test/Subsystem");
        testControllerChooser.addDefaultOption("", ""); // Add default option so code doesn't crash on read

        if (Constants.swerveEnabled) {
            testControllerChooser.addOption("Swerve", "Swerve");

            // Test controller swerve control for convenience
            testController
                    .axisMagnitudeGreaterThan(2, Swerve.Constants.turnDeadband)
                    .or(() -> Math.hypot(testController.getLeftX(), testController.getLeftY())
                            > Swerve.Constants.moveDeadband)
                    .onTrue(swerveCommands.drive(
                            () -> -testController.getLeftY(),
                            () -> -testController.getLeftX(),
                            () -> -testController.getRightX(),
                            () -> Swerve.Constants.swerveFieldCentric.get()));

            // Manual duty cycle forward test
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
                    .and(() -> testControllerChooser.get().equals("Swerve"))
                    .onTrue(swerveCommands.setSpeed(-0.2, 0, 0))
                    .onFalse(swerveCommands.stop());

            // Manual duty cycle forward test, fast
            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("Swerve"))
                    .onTrue(swerveCommands.setSpeed(1, 0, 0))
                    .onFalse(swerveCommands.stop());

            // Manual duty cycle backward test, fast
            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("Swerve"))
                    .onTrue(swerveCommands.setSpeed(-1, 0, 0))
                    .onFalse(swerveCommands.stop());
        }

        if (Constants.hangEnabled) {
            testControllerChooser.addOption("Hang", "Hang");
            // Hang move up test
            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Manual"))
                    .and(() -> testControllerChooser.get().equals("Hang"))
                    .whileTrue(hangCommands.setSpeed(() -> 0.1));

            // Hang move down test
            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Manual"))
                    .and(() -> testControllerChooser.get().equals("Hang"))
                    .whileTrue(hangCommands.setSpeed(() -> -0.1));

            // Hang move up test
            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("Hang"))
                    .whileTrue(hangCommands.moveUp());

            // Hang move down test
            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("Hang"))
                    .whileTrue(hangCommands.moveDown());
        }

        if (Constants.hopperEnabled) {
            testControllerChooser.addOption("Hopper", "Hopper");

            // Hopper slow forward test
            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Manual"))
                    .and(() -> testControllerChooser.get().equals("Hopper"))
                    .whileTrue(hopperCommands.setSpeed(() -> 0.1));

            // Hopper slow reverse test
            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Manual"))
                    .and(() -> testControllerChooser.get().equals("Hopper"))
                    .whileTrue(hopperCommands.setSpeed(() -> -0.1));

            // Hopper fast forward test
            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("Hopper"))
                    .whileTrue(hopperCommands.forward());

            // Hopper fast reverse test
            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("Hopper"))
                    .whileTrue(hopperCommands.reverse());
        }

        if (Constants.shooterEnabled) {
            testControllerChooser.addOption("ShooterFeed", "ShooterFeed");
            testControllerChooser.addOption("ShooterFly", "ShooterFly");

            // Slow flywheel forward test
            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Manual"))
                    .and(() -> testControllerChooser.get().equals("ShooterFly"))
                    .whileTrue(shooterCommands.setFlySpeed(() -> 0.1));

            // Slow flywheel reverse test
            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Manual"))
                    .and(() -> testControllerChooser.get().equals("ShooterFly"))
                    .whileTrue(shooterCommands.setFlySpeed(() -> -0.1));

            // Fast flywheel forward test
            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("ShooterFly"))
                    .whileTrue(shooterCommands.flyShoot());

            // Fast flywheel reverse test
            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("ShooterFly"))
                    .whileTrue(shooterCommands.flyReverse());

            // Slow feed forward test
            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Manual"))
                    .and(() -> testControllerChooser.get().equals("ShooterFeed"))
                    .whileTrue(shooterCommands.setFeedSpeed(() -> 0.1));

            // Slow feed reverse test
            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Manual"))
                    .and(() -> testControllerChooser.get().equals("ShooterFeed"))
                    .whileTrue(shooterCommands.setFeedSpeed(() -> -0.1));

            // Fast feed forward test
            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("ShooterFeed"))
                    .whileTrue(shooterCommands.feedShoot());

            // Fast feed reverse test
            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("ShooterFeed"))
                    .whileTrue(shooterCommands.feedReverse());
        }

        if (Constants.intakeEnabled) {
            testControllerChooser.addOption("Intake", "Intake");
            testControllerChooser.addOption("IntakeHinge", "IntakeHinge");

            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Manual"))
                    .and(() -> testControllerChooser.get().equals("Intake"))
                    .whileTrue(intakeCommands.setIntakeSpeed(() -> 0.1));

            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Manual"))
                    .and(() -> testControllerChooser.get().equals("Intake"))
                    .whileTrue(intakeCommands.setIntakeSpeed(() -> -0.1));

            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("Intake"))
                    .whileTrue(intakeCommands.intake());

            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("Intake"))
                    .whileTrue(intakeCommands.outtake());

            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Manual"))
                    .and(() -> testControllerChooser.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.setHingeSpeed(() -> 0.1));

            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Manual"))
                    .and(() -> testControllerChooser.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.setHingeSpeed(() -> -0.1));

            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.setHingeSpeed(() -> 0.5));

            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.setHingeSpeed(() -> -0.5));

            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("PID"))
                    .and(() -> testControllerChooser.get().equals("IntakeHinge"))
                    .onTrue(intakeCommands.hingeUp());

            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("PID"))
                    .and(() -> testControllerChooser.get().equals("IntakeHinge"))
                    .onTrue(intakeCommands.hingeDown());

            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("PIDChange"))
                    .and(() -> testControllerChooser.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.changeGoal(() -> 0.02));

            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("PIDChange"))
                    .and(() -> testControllerChooser.get().equals("IntakeHinge"))
                    .whileTrue(intakeCommands.changeGoal(() -> -0.02));
        }
    }

    // Bindings for manual control of each of the subsystems (nothing here for swerve, add other subsystems)
    public void configureManualBindings() {}

    // Refresh drive and operator disconnect alerts
    public void refreshControllerAlerts() {
        controllerDisconnected.set(!driveController.isConnected());
        operatorDisconnected.set(!operator.isConnected());
    }

    // Initialize dashboard auto chooser
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
        } else {
            Alerts.create("Unknown auto specified", AlertType.kWarning);
            return new InstantCommand();
        }
    }

    public void periodic() {
        if (Constants.swerveEnabled) {
            publisher.publish(); // Publish 3D robot data
        }
        refreshControllerAlerts(); // Enable alerts for controller disconnects
    }
}
