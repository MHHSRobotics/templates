package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.io.GyroIO;
import frc.robot.util.Field;

// Simulates the overall tank drive robot pose based on wheel simulations
public class DriveSim extends SubsystemBase {
    private DifferentialDriveKinematics kinematics;

    private DriveWheelSim leftWheelSim;
    private DriveWheelSim rightWheelSim;
    private GyroIO gyro;

    // Simulated robot pose (actual physical position in simulation)
    private Pose2d currentPose = new Pose2d(Field.fieldLength / 2, Field.fieldWidth / 2, Rotation2d.kZero);

    // Track heading for gyro simulation
    private double currentHeadingRad = 0;

    public DriveSim(DriveWheelSim leftWheelSim, DriveWheelSim rightWheelSim, GyroIO gyro) {
        this.leftWheelSim = leftWheelSim;
        this.rightWheelSim = rightWheelSim;
        this.gyro = gyro;
        this.kinematics = new DifferentialDriveKinematics(Drive.Constants.trackWidthMeters);
    }

    // Get the actual physical pose of the robot in simulation
    public Pose2d getPhysicalPose() {
        return currentPose;
    }

    // Get the simulated heading in radians
    public double getHeadingRad() {
        return currentHeadingRad;
    }

    @Override
    public void periodic() {
        // Get wheel velocities from wheel simulations
        double leftVelocity = leftWheelSim.getLinearVelocityMetersPerSec();
        double rightVelocity = rightWheelSim.getLinearVelocityMetersPerSec();

        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);

        // Convert wheel speeds to chassis speeds
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

        // Integrate chassis speeds to get pose change
        Twist2d twist = chassisSpeeds.toTwist2d(Constants.loopTime);

        // Update pose
        currentPose = currentPose.exp(twist);

        // Update heading
        currentHeadingRad = currentPose.getRotation().getRadians();

        // Update gyro simulation with current heading
        gyro.setMechYaw(currentHeadingRad);
        gyro.setMechYawVelocity(chassisSpeeds.omegaRadiansPerSecond);

        // Log the actual simulated pose
        Logger.recordOutput("DriveSim/ActualPose", currentPose);
        Logger.recordOutput("DriveSim/LeftVelocityMPS", leftVelocity);
        Logger.recordOutput("DriveSim/RightVelocityMPS", rightVelocity);
    }
}
