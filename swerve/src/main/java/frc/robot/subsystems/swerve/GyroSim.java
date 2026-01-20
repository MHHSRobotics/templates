package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.io.GyroIO;

// Gyro sim does NOTHING because swerve replaces gyro with odometry when it's disconnected
public class GyroSim extends SubsystemBase {
    private GyroIO gyro;

    public GyroSim(GyroIO gyroIO) {
        gyro = gyroIO;
    }

    @Override
    public void periodic() {
        gyro.disconnect();
    }
}
