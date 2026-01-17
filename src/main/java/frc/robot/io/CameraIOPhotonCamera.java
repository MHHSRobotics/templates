package frc.robot.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert.AlertType;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.Alerts;
import frc.robot.util.Field;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraIOPhotonCamera extends CameraIO {
    private PhotonCamera cam;

    private Transform3d robotToCamera;

    private double fov;

    public CameraIOPhotonCamera(String name, String logPath, Transform3d robotToCamera, double fov) {
        super(name, logPath);
        cam = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
        this.fov = fov;
    }

    @Override
    public void update() {
        inputs.connected = cam.isConnected();
        inputs.measurements = 0;

        var unreadResults = cam.getAllUnreadResults();
        for (PhotonPipelineResult res : unreadResults) {
            if (inputs.measurements < Swerve.VisionConstants.maxMeasurements && res.hasTargets()) {
                // Try multi-tag first (most accurate when multiple tags are visible)
                var multiTagResult = res.getMultiTagResult();
                if (multiTagResult.isPresent()) {
                    var result = multiTagResult.get();
                    // Multi-tag result gives field-to-camera transform, convert to camera pose then to robot pose
                    Pose3d estimatedRobotPose =
                            new Pose3d().transformBy(result.estimatedPose.best).transformBy(robotToCamera.inverse());
                    inputs.poses[inputs.measurements] = estimatedRobotPose;
                    inputs.poseTimestamps[inputs.measurements] = res.getTimestampSeconds();
                    inputs.ambiguities[inputs.measurements] = result.estimatedPose.ambiguity;
                    inputs.tagCounts[inputs.measurements] = result.fiducialIDsUsed.size();
                    inputs.measurements++;
                } else {
                    // Fall back to single-tag if multi-tag unavailable
                    PhotonTrackedTarget target = res.getBestTarget();
                    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                    var aprilTagPose = Field.layout.getTagPose(target.getFiducialId());
                    if (aprilTagPose.isPresent()) {
                        Pose3d aprilTagLocation = aprilTagPose.get();
                        Pose3d estimatedRobotPose = aprilTagLocation
                                .transformBy(bestCameraToTarget.inverse())
                                .transformBy(robotToCamera.inverse());
                        inputs.poses[inputs.measurements] = estimatedRobotPose;
                        inputs.poseTimestamps[inputs.measurements] = res.getTimestampSeconds();
                        inputs.ambiguities[inputs.measurements] = target.getPoseAmbiguity();
                        inputs.tagCounts[inputs.measurements] = 1;
                        inputs.measurements++;
                    }
                }
            }
        }

        for (int i = inputs.measurements; i < Swerve.VisionConstants.maxMeasurements; i++) {
            inputs.poses[i] = new Pose3d();
        }
        super.update();
    }

    @Override
    public void startSim(VisionSystemSim visionSim) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method startSim on " + getName(), AlertType.kWarning);
            return;
        }
        // Config for the camera sim
        SimCameraProperties cameraProp = new SimCameraProperties();
        // 640x480 input with 80 degree FOV
        cameraProp.setCalibration(320, 240, Rotation2d.fromDegrees(fov));
        // 50 FPS
        cameraProp.setFPS(50);
        // Latency of 35ms with standard deviation of 5ms
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        PhotonCameraSim cameraSim = new PhotonCameraSim(cam, cameraProp);
        visionSim.addCamera(cameraSim, robotToCamera);

        cameraSim.enableDrawWireframe(true);
    }
}
