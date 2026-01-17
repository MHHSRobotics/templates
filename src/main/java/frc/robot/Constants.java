package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

import com.ctre.phoenix6.CANBus;

public final class Constants {
    // Should be Mode.REPLAY when replaying, else Mode.SIM
    public static final Mode simMode = Mode.SIM;

    // Current mode the robot program is in
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY,

        // WIP physics simulator
        PHYSICS_SIM,
    }

    // Period of main robot loop
    public static final double loopTime=0.02;

    public static final CANBus defaultBus = new CANBus("rio");
    public static final CANBus swerveBus = new CANBus("rhino");

    public static final double loopOverrunWarningTimeout = 0.2;

    // Voltage at which brownout protection occurs
    public static final double brownoutVoltage = 6.0;

    // Voltage at which low battery warning appears
    public static final double lowBatteryVoltage = 11.8;

    // How long to wait before reporting low battery
    public static final double lowBatteryTime = 0.5;

    // Whether simulation should be on red alliance
    public static final boolean simIsRedAlliance = false;

    public static final boolean swerveEnabled = true;
    public static final boolean visionEnabled = true;
}
