package frc.robot.network;

import edu.wpi.first.math.geometry.Pose3d;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.swerve.Swerve;

// Class that publishes 3D robot data to AdvantageScope
public class RobotPublisher {

    private Swerve swerve;

    public RobotPublisher(Swerve swerve) {
        this.swerve = swerve;
    }

    public void publish() {
        // Record 3D position of bot in AScope
        Logger.recordOutput("3DField/Chassis", new Pose3d(swerve.getPose()));
    }
}
