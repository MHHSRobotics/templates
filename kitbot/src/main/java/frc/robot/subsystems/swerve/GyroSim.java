package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.io.GyroIO;

// Gyro sim does NOTHING because swerve replaces gyro with odometry when it's disconnected
public class GyroSim extends SubsystemBase {
    private GyroIO gyro;
    private SwerveSim swerveSim;

    public GyroSim(GyroIO gyroIO, SwerveSim swerveSim) {
        gyro = gyroIO;
        this.swerveSim = swerveSim;
    }

    @Override
    public void periodic() {
        gyro.setMechYaw(swerveSim.getPhysicalPose().getRotation().getRadians());
        gyro.setMechYawVelocity(0);
    }
}
