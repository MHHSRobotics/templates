package frc.robot.network;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.drive.Drive;

// Class that publishes 3D robot data to AdvantageScope
public class RobotPublisher {

    private Drive drive;

    public RobotPublisher(Drive drive) {
        this.drive = drive;
    }

    public void publish() {
        Pose2d pos = drive.getPose();
        // Rotate 180 degrees because the orientation of the robot model is wrong
        pos = pos.transformBy(new Transform2d(new Translation2d(), Rotation2d.k180deg));
        Pose3d botPos = new Pose3d(pos);

        // Position of the main link needs to be absolute, so add botPos to chassisPos for main link
        Transform3d chassisPos = new Transform3d(0, 0, 0.04, new Rotation3d());
        Logger.recordOutput("3DField/Chassis", botPos.plus(chassisPos));

        // Convert chassisPos to Pose3d, other links need relative positions
        Pose3d mainPos = new Pose3d(chassisPos.getTranslation(), chassisPos.getRotation());

        // Additional mechanism poses can be added here for visualization
    }
}
