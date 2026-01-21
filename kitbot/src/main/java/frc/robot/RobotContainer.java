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

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FuelCommands;
import frc.robot.io.GyroIO;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIOSparkMax;
import frc.robot.network.RobotPublisher;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveSim;
import frc.robot.subsystems.drive.DriveWheelSim;
import frc.robot.subsystems.fuel.Fuel;
import frc.robot.util.Alerts;

public class RobotContainer {
    // Subsystems
    private Drive drive;
    private Fuel fuel;

    // Subsystem commands
    private DriveCommands driveCommands;
    private FuelCommands fuelCommands;

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

        if (Constants.driveEnabled) {
            publisher = new RobotPublisher(drive); // Initialize the 3D data publisher
        }
    }

    private void initSubsystems() {
        // Initialize tank drive motors and gyro
        if (Constants.driveEnabled) {
            // Create variables for drive motors (front = leaders, back = followers) and gyro
            MotorIO leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
            GyroIO gyro;

            switch (Constants.currentMode) {
                // If in REAL or SIM mode, use MotorIOSparkMax for motors
                case REAL:
                case SIM:
                    // Create all 4 drive motors - CIM motors are brushed
                    leftFrontMotor = new MotorIOSparkMax(
                            Drive.Constants.leftFrontMotorId,
                            MotorType.kBrushed,
                            "left front drive motor",
                            "Drive/LeftFront");
                    leftBackMotor = new MotorIOSparkMax(
                            Drive.Constants.leftBackMotorId,
                            MotorType.kBrushed,
                            "left back drive motor",
                            "Drive/LeftBack");
                    rightFrontMotor = new MotorIOSparkMax(
                            Drive.Constants.rightFrontMotorId,
                            MotorType.kBrushed,
                            "right front drive motor",
                            "Drive/RightFront");
                    rightBackMotor = new MotorIOSparkMax(
                            Drive.Constants.rightBackMotorId,
                            MotorType.kBrushed,
                            "right back drive motor",
                            "Drive/RightBack");

                    // No gyro on kitbot - use empty GyroIO (Drive will use wheel odometry for heading)
                    gyro = new GyroIO("gyro", "Drive/Gyro");
                    break;
                default:
                    // If in REPLAY, use empty MotorIO objects
                    leftFrontMotor = new MotorIO("left front drive motor", "Drive/LeftFront");
                    leftBackMotor = new MotorIO("left back drive motor", "Drive/LeftBack");
                    rightFrontMotor = new MotorIO("right front drive motor", "Drive/RightFront");
                    rightBackMotor = new MotorIO("right back drive motor", "Drive/RightBack");
                    gyro = new GyroIO("gyro", "Drive/Gyro");
                    break;
            }

            // Initialize drive subsystem with all 4 motors (Drive configures followers internally)
            drive = new Drive(gyro, leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);

            // If mode is SIM, start the simulations for drive wheels and overall drive
            if (Constants.currentMode == Mode.SIM) {
                // Only simulate the leader motors - followers copy their output
                DriveWheelSim leftWheelSim = new DriveWheelSim(leftFrontMotor);
                DriveWheelSim rightWheelSim = new DriveWheelSim(rightFrontMotor);
                new DriveSim(leftWheelSim, rightWheelSim, gyro);
            }
        }

        // Initialize fuel subsystem motors
        if (Constants.fuelEnabled) {
            MotorIO intakeLauncherMotor, feederMotor;

            switch (Constants.currentMode) {
                case REAL:
                case SIM:
                    // Create fuel motors - brushed motors
                    intakeLauncherMotor = new MotorIOSparkMax(
                            Fuel.Constants.intakeLauncherMotorId,
                            MotorType.kBrushed,
                            "intake launcher motor",
                            "Fuel/IntakeLauncher");
                    feederMotor = new MotorIOSparkMax(
                            Fuel.Constants.feederMotorId, MotorType.kBrushed, "feeder motor", "Fuel/Feeder");
                    break;
                default:
                    // If in REPLAY, use empty MotorIO objects
                    intakeLauncherMotor = new MotorIO("intake launcher motor", "Fuel/IntakeLauncher");
                    feederMotor = new MotorIO("feeder motor", "Fuel/Feeder");
                    break;
            }

            // Initialize fuel subsystem
            fuel = new Fuel(intakeLauncherMotor, feederMotor);
        }
    }

    private void initCommands() {
        if (Constants.driveEnabled) {
            driveCommands = new DriveCommands(drive);
        }
        if (Constants.fuelEnabled) {
            fuelCommands = new FuelCommands(fuel);
        }
    }

    private void configureBindings() {
        /* ---- Main controller bindings ---- */
        /*
         * Reset gyro: create
         * Left stick Y: forward/backward
         * Right stick X: turn
         * Touchpad: cancel all commands
         */

        controller.touchpad().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance()
                .cancelAll()));

        if (Constants.driveEnabled) {
            controller.create().onTrue(driveCommands.resetGyro());

            /*
             * Arcade drive using left stick Y for forward/backward and right stick X for turning.
             * The command runs continuously when either stick is outside its deadband.
             */
            controller
                    .axisMagnitudeGreaterThan(0, Drive.Constants.turnDeadband)
                    .or(controller.axisMagnitudeGreaterThan(1, Drive.Constants.moveDeadband))
                    .onTrue(driveCommands.arcadeDrive(() -> -controller.getLeftY(), () -> -controller.getRightX()));
        }

        // Fuel subsystem bindings
        if (Constants.fuelEnabled) {
            // L1: Intake - runs while held
            controller.L1().whileTrue(fuelCommands.intake());

            // R1: Launch sequence - spin up then launch, runs while held
            controller.R1().whileTrue(fuelCommands.launchSequence());

            // Cross (X): Eject - runs while held
            controller.cross().whileTrue(fuelCommands.eject());
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
        testControllerChooser.addOption("Drive", "Drive");

        testControllerManual = new LoggedDashboardChooser<>("Test/Type");
        testControllerManual.addOption("Manual", "Manual");
        testControllerManual.addOption("PID", "PID");
        testControllerManual.addOption("Fast", "Fast");

        if (Constants.driveEnabled) {
            // Test controller drive control for convenience
            testController
                    .axisMagnitudeGreaterThan(0, Drive.Constants.turnDeadband)
                    .or(testController.axisMagnitudeGreaterThan(1, Drive.Constants.moveDeadband))
                    .onTrue(driveCommands.arcadeDrive(
                            () -> -testController.getLeftY(), () -> -testController.getRightX()));

            // Manual duty cycle forward test
            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Manual"))
                    .and(() -> testControllerChooser.get().equals("Drive"))
                    .onTrue(driveCommands.setArcadeSpeed(0.2, 0))
                    .onFalse(driveCommands.stop());

            // Manual duty cycle backward test
            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Manual"))
                    .and(() -> testControllerChooser.get().equals("Drive"))
                    .onTrue(driveCommands.setArcadeSpeed(-0.2, 0))
                    .onFalse(driveCommands.stop());

            // Manual duty cycle forward test, fast
            testController
                    .cross()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("Drive"))
                    .onTrue(driveCommands.setArcadeSpeed(1, 0))
                    .onFalse(driveCommands.stop());

            // Manual duty cycle backward test, fast
            testController
                    .circle()
                    .and(() -> testControllerManual.get().equals("Fast"))
                    .and(() -> testControllerChooser.get().equals("Drive"))
                    .onTrue(driveCommands.setArcadeSpeed(-1, 0))
                    .onFalse(driveCommands.stop());
        }
    }

    // Bindings for manual control of each of the subsystems (nothing here for drive, add other subsystems)
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
            // Simple auto: drive forward for 2 seconds then stop
            return driveCommands
                    .setArcadeSpeed(0.3, 0)
                    .andThen(new WaitCommand(2))
                    .andThen(driveCommands.stop());
        } else {
            Alerts.create("Unknown auto specified", AlertType.kWarning);
            return new InstantCommand();
        }
    }

    public void periodic() {
        if (Constants.driveEnabled) {
            publisher.publish(); // Publish 3D robot data
        }
        refreshControllerAlerts(); // Enable alerts for controller disconnects
    }
}
