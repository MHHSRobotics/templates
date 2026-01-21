package frc.robot.subsystems.fuel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.io.MotorIO;

// Fuel subsystem for the kitbot intake/launcher mechanism.
// Uses two rollers:
// - Intake/Launcher roller: handles both intaking and launching fuel
// - Feeder roller: feeds fuel from intake to launcher
public class Fuel extends SubsystemBase {
    public static class Constants {
        // Motor CAN IDs
        public static final int intakeLauncherMotorId = 5;
        public static final int feederMotorId = 6;

        // Current limits (amps)
        public static final int motorCurrentLimit = 60;

        // Voltage values for various fuel operations (tunable via NetworkTables)
        public static final LoggedNetworkNumber intakingFeederVoltage =
                new LoggedNetworkNumber("Fuel/IntakingFeederVoltage", -12);
        public static final LoggedNetworkNumber intakingIntakeVoltage =
                new LoggedNetworkNumber("Fuel/IntakingIntakeVoltage", 10);
        public static final LoggedNetworkNumber launchingFeederVoltage =
                new LoggedNetworkNumber("Fuel/LaunchingFeederVoltage", 9);
        public static final LoggedNetworkNumber launchingLauncherVoltage =
                new LoggedNetworkNumber("Fuel/LaunchingLauncherVoltage", 10.6);
        public static final LoggedNetworkNumber spinUpFeederVoltage =
                new LoggedNetworkNumber("Fuel/SpinUpFeederVoltage", -6);

        // Spin-up time before launching (seconds)
        public static final double spinUpSeconds = 1.0;
    }

    // Motor controllers
    private MotorIO intakeLauncherMotor;
    private MotorIO feederMotor;

    public Fuel(MotorIO intakeLauncherMotor, MotorIO feederMotor) {
        this.intakeLauncherMotor = intakeLauncherMotor;
        this.feederMotor = feederMotor;

        // Configure motors
        configureMotors();
    }

    private void configureMotors() {
        // Set current limits
        intakeLauncherMotor.setSupplyCurrentLimit(Constants.motorCurrentLimit);
        feederMotor.setSupplyCurrentLimit(Constants.motorCurrentLimit);

        // Intake/launcher motor is inverted so positive values work for both intake and launch
        intakeLauncherMotor.setInverted(true);

        // Enable braking
        intakeLauncherMotor.setBraking(true);
        feederMotor.setBraking(true);
    }

    // Set the intake/launcher roller voltage
    public void setIntakeLauncherVoltage(double voltage) {
        intakeLauncherMotor.setVoltage(voltage);
    }

    // Set the feeder roller voltage
    public void setFeederVoltage(double voltage) {
        feederMotor.setVoltage(voltage);
    }

    // Set both rollers for intaking
    public void intake() {
        setIntakeLauncherVoltage(Constants.intakingIntakeVoltage.get());
        setFeederVoltage(Constants.intakingFeederVoltage.get());
    }

    // Set both rollers for ejecting (reverse of intake)
    public void eject() {
        setIntakeLauncherVoltage(-Constants.intakingIntakeVoltage.get());
        setFeederVoltage(-Constants.intakingFeederVoltage.get());
    }

    // Set rollers for spin-up (launcher at full, feeder at reduced power)
    public void spinUp() {
        setIntakeLauncherVoltage(Constants.launchingLauncherVoltage.get());
        setFeederVoltage(Constants.spinUpFeederVoltage.get());
    }

    // Set both rollers for launching
    public void launch() {
        setIntakeLauncherVoltage(Constants.launchingLauncherVoltage.get());
        setFeederVoltage(Constants.launchingFeederVoltage.get());
    }

    // Stop both rollers
    public void stop() {
        intakeLauncherMotor.setVoltage(0);
        feederMotor.setVoltage(0);
    }

    // Get the intake/launcher motor (for simulation)
    public MotorIO getIntakeLauncherMotor() {
        return intakeLauncherMotor;
    }

    // Get the feeder motor (for simulation)
    public MotorIO getFeederMotor() {
        return feederMotor;
    }

    @Override
    public void periodic() {
        // Update motor inputs
        intakeLauncherMotor.update();
        feederMotor.update();

        // Log data
        Logger.recordOutput("Fuel/IntakeLauncherVoltage", intakeLauncherMotor.getInputs().appliedVoltage);
        Logger.recordOutput("Fuel/FeederVoltage", feederMotor.getInputs().appliedVoltage);
        Logger.recordOutput("Fuel/IntakeLauncherCurrent", intakeLauncherMotor.getInputs().supplyCurrent);
        Logger.recordOutput("Fuel/FeederCurrent", feederMotor.getInputs().supplyCurrent);
    }
}
