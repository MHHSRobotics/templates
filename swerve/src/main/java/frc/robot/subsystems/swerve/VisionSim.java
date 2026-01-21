package frc.robot.subsystems.swerve;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.io.CameraIO;
import frc.robot.util.Field;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSim extends SubsystemBase {
    private VisionSystemSim visionSim;
    private SwerveSim swerveSim;

    public VisionSim(List<CameraIO> cameras, SwerveSim swerveSim) {
        this.swerveSim = swerveSim;
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(Field.layout);

        for (CameraIO cam : cameras) {
            cam.startSim(visionSim);
        }
    }

    @Override
    public void periodic() {
        visionSim.update(swerveSim.getPhysicalPose());
    }
}
