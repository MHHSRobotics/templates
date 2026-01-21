package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;

public class SwerveSim extends SubsystemBase {
    private SwerveDriveKinematics kinematics;

    private SwerveModuleSim[] moduleSims;

    private Pose2d currentPose = Swerve.Constants.simInitialPose.get();

    public SwerveSim(SwerveModuleSim[] moduleSims) {
        this.moduleSims = moduleSims;
        kinematics = new SwerveDriveKinematics(Swerve.getModuleTranslations());
    }

    public Pose2d getPhysicalPose() {
        return currentPose;
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = moduleSims[i].getState();

            states[i].angle = states[i].angle.plus(Rotation2d.fromRotations(
                    (Math.random() * 2 - 1) * Constants.simSwerveError)); // Random angle error for simulation
            states[i].speedMetersPerSecond *=
                    (1 + (Math.random() * 2 - 1) * Constants.simSwerveError); // Random speed error for simulation
        }
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(states);
        Twist2d twist = speeds.toTwist2d(Constants.loopTime);
        currentPose = currentPose.exp(twist);
        Logger.recordOutput("SwerveSim/ActualPose", currentPose);
    }
}
