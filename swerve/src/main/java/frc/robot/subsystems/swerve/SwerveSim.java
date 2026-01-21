package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        }
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(states);
        Twist2d twist = speeds.toTwist2d(Constants.loopTime);
        currentPose = currentPose.exp(twist);
    }
}
