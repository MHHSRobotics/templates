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
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.swerve.VisionSim;
import frc.robot.util.Alerts;

public class RobotContainer {
    private Swerve swerve;

    private SwerveCommands swerveCommands;

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
        publisher = new RobotPublisher(swerve);
    }

    private void initSubsystems() {
        // Initialize subsystems in order: arm, elevator, wrist, intake, hang, swerve
        // Each subsystem is created immediately after its motor/encoder initialization

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
        swerveCommands = new SwerveCommands(swerve);
    }

    private void configureBindings() {
        /* ---- Main controller bindings ---- */

        // controller.R2().onTrue(ssCommands.lowAlgaePosition());

        controller.create().onTrue(swerveCommands.resetGyro());

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
    }

    // Bindings for manual control of each of the subsystems
    public void configureManualBindings() {
        // Square + circle control arm

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
