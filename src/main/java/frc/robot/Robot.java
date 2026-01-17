package frc.robot;

import java.io.File;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.SignalLogger;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;

    private final Timer lowBatteryTimer = new Timer();

    private final Alert lowBatteryAlert = new Alert("Battery charge is low, replace it soon", AlertType.kWarning);

    public Robot() {
        // Add the project metadata to the logs so we can identify which version of the code created a specific log file
        Logger.recordMetadata("Name", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set logging mode depending on the current running mode
        switch (Constants.currentMode) {
            case REAL:
                // Check if log file exists
                File file = new File("/U/logs");
                if (!file.exists()) {
                    Alerts.create("Log USB drive not found!", AlertType.kWarning);
                }
            case SIM:
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            default:
                setUseTiming(false);
                String logPath =
                        LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
                break;
        }

        Logger.start();

        // Disable automatic Hoot logging
        SignalLogger.enableAutoLogging(false);

        // Adjust loop overrun warning timeout
        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(Constants.loopOverrunWarningTimeout);
        } catch (Exception e) {
            Alerts.create("Failed to disable loop overrun warnings", AlertType.kWarning);
        }
        CommandScheduler.getInstance().setPeriod(Constants.loopOverrunWarningTimeout);

        // Remove controller connection warnings
        DriverStation.silenceJoystickConnectionWarning(true);

        // Log active commands
        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger.recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
        CommandScheduler.getInstance()
                .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
        CommandScheduler.getInstance().onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
        CommandScheduler.getInstance()
                .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

        // Configure brownout voltage
        RobotController.setBrownoutVoltage(Constants.brownoutVoltage);

        // Restart timers
        lowBatteryTimer.restart();

        // Init robot container
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // Run all active commands and call periodic() on subsystems
        CommandScheduler.getInstance().run();

        if (RobotController.getBatteryVoltage() <= Constants.lowBatteryVoltage && DriverStation.isEnabled()) {
            if (lowBatteryTimer.hasElapsed(Constants.lowBatteryTime)) {
                lowBatteryAlert.set(true);
            }
        } else {
            lowBatteryAlert.set(false);
            lowBatteryTimer.reset();
        }

        // Log CANivore status
        CANBusStatus status = Constants.swerveBus.getStatus();
        Logger.recordOutput("CANivore/Utilization", status.BusUtilization);
        Logger.recordOutput("CANivore/Status", status.Status.isOK());

        // RobotContainer periodic gets called _after_ the subsystems
        robotContainer.periodic();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        if (Constants.currentMode == Mode.SIM) {
            // Choose alliance station based on Constants.simIsRedAlliance
            if (Constants.simIsRedAlliance) {
                DriverStationSim.setAllianceStationId(AllianceStationID.Red2);
            } else {
                DriverStationSim.setAllianceStationId(AllianceStationID.Blue2);
            }
        }

        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (Constants.currentMode == Mode.SIM) {
            // Choose alliance station based on Constants.simIsRedAlliance
            if (Constants.simIsRedAlliance) {
                DriverStationSim.setAllianceStationId(AllianceStationID.Red2);
            } else {
                DriverStationSim.setAllianceStationId(AllianceStationID.Blue2);
            }
        }

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
