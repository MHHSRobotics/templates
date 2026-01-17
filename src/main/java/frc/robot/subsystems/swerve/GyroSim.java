package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.io.GyroIO;

// Simple gyro sim that disconnects the gyro so odometry is used in sim
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
