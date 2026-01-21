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
import frc.robot.commands.SwerveCommands;
import frc.robot.io.CameraIO;
import frc.robot.io.CameraIOPhotonCamera;
import frc.robot.io.EncoderIO;
import frc.robot.io.EncoderIOCANcoder;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIOTalonFX;
import frc.robot.network.RobotPublisher;
import frc.robot.subsystems.swerve.GyroSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleSim;
import frc.robot.subsystems.swerve.SwerveSim;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.swerve.VisionSim;
import frc.robot.util.Alerts;

public class RobotContainer {
    // Subsystems
    private Swerve swerve;

    // Subsystem commands
    private SwerveCommands swerveCommands;

    private final CommandPS5Controller controller = new CommandPS5Controller(0); // Main drive controller

    private final CommandPS5Controller manualController =
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
    private Alert manualDisconnected = new Alert("Manual controller is disconnected", AlertType.kWarning);

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
        // Initialize swerve motors, encoders, and gyro
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
                CameraIO brat;
                CameraIO blat;
                switch (Constants.currentMode) {
                    case REAL:
                    case SIM:
                        // If in real bot or sim, use CameraIOPhotonCamera
                        brat = new CameraIOPhotonCamera(
                                "BackRight_AT", "Vision/BRAT", Swerve.VisionConstants.bratPose, 60);
                        blat = new CameraIOPhotonCamera(
                                "BackLeft_AT", "Vision/BLAT", Swerve.VisionConstants.blatPose, 60);
                        break;
                    default:
                        // If in replay use an empty CameraIO
                        brat = new CameraIO("BackRight_AT", "Vision/BRAT");
                        blat = new CameraIO("BackLeft_AT", "Vision/BLAT");
                        break;
                }
                // Add cameras to swerve ododmetry
                swerve.addCameraSource(brat);
                swerve.addCameraSource(blat);
            }

            // If mode is SIM, start the simulations for swerve modules and gyro
            if (Constants.currentMode == Mode.SIM) {
                SwerveModuleSim[] moduleSims = new SwerveModuleSim[] {
                    new SwerveModuleSim(flDriveMotor, flAngleMotor, flEncoder, TunerConstants.FrontLeft),
                    new SwerveModuleSim(frDriveMotor, frAngleMotor, frEncoder, TunerConstants.FrontRight),
                    new SwerveModuleSim(blDriveMotor, blAngleMotor, blEncoder, TunerConstants.BackLeft),
                    new SwerveModuleSim(brDriveMotor, brAngleMotor, brEncoder, TunerConstants.BackRight)
                };

                SwerveSim swerveSim = new SwerveSim(moduleSims);

                new GyroSim(gyro, swerveSim);
                if (Constants.visionEnabled) {
                    new VisionSim(swerve.getCameras(), swerveSim);
                }
            }
        }
    }

    private void initCommands() {
        if (Constants.swerveEnabled) {
            swerveCommands = new SwerveCommands(swerve);
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
            controller.create().onTrue(swerveCommands.resetGyro());

            /*
             * How this works:
             * When the driver controller is outside of its deadband, it runs swerveCommands.drive(), which overrides auto align commands. swerveCommands.drive() will continue to run until an auto align command is executed, so the swerve drive will stop when both sticks are at 0.
             */
            controller
                    .axisMagnitudeGreaterThan(2, Swerve.Constants.turnDeadband)
                    .or(() -> Math.hypot(controller.getLeftX(), controller.getLeftY()) > Swerve.Constants.moveDeadband)
                    .onTrue(swerveCommands.drive(
                            () -> -controller.getLeftY(),
                            () -> -controller.getLeftX(),
                            () -> -controller.getRightX(),
                            () -> Swerve.Constants.swerveFieldCentric.get()));

            controller.touchpad().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance()
                    .cancelAll()));
        }
    }

    private void configureTestBindings() {
        /* ---- Test controller bindings ---- */
        /*
         * Forward manual/PID: cross
         * Backward manual/PID: circle
         */
        // Initialize dashboard choosers
        testControllerChooser = new LoggedDashboardChooser<>("Test/Subsystem");
        testControllerChooser.addOption("Swerve", "Swerve");

        testControllerManual = new LoggedDashboardChooser<>("Test/Type");
        testControllerManual.addOption("Manual", "Manual");
        testControllerManual.addOption("PID", "PID");
        testControllerManual.addOption("Fast", "Fast");

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

    // Bindings for manual control of each of the subsystems (nothing here for swerve, add other subsystems)
    public void configureManualBindings() {}

    // Refresh drive and manual controller disconnect alerts
    public void refreshControllerAlerts() {
        controllerDisconnected.set(!controller.isConnected());
        manualDisconnected.set(!manualController.isConnected());
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
        publisher.publish(); // Publish 3D robot data
        refreshControllerAlerts(); // Enable alerts for controller disconnects
    }
}
