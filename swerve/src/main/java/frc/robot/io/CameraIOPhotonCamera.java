package frc.robot.io;

import java.util.ArrayList;
import java.util.List;

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
    private int resWidth, resHeight;

    private boolean disconnected = false;

    public CameraIOPhotonCamera(
            String name, String logPath, Transform3d robotToCamera, double fov, int resWidth, int resHeight) {
        super(name, logPath);
        cam = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
        this.fov = fov;
        this.resWidth = resWidth;
        this.resHeight = resHeight;
    }

    // Camera resolution defaults to 320x240
    public CameraIOPhotonCamera(String name, String logPath, Transform3d robotToCamera, double fov) {
        this(name, logPath, robotToCamera, fov, 320, 240);
    }

    @Override
    public void update() {
        inputs.connected = disconnected ? false : cam.isConnected();

        List<Pose3d> posesList = new ArrayList<>();
        List<Double> timestampsList = new ArrayList<>();
        List<Double> ambiguitiesList = new ArrayList<>();
        List<Integer> tagCountsList = new ArrayList<>();

        var unreadResults = cam.getAllUnreadResults();
        for (PhotonPipelineResult res : unreadResults) {
            if (posesList.size() < Swerve.VisionConstants.maxMeasurements && res.hasTargets()) {
                // Try multi-tag first (most accurate when multiple tags are visible)
                var multiTagResult = res.getMultiTagResult();
                if (multiTagResult.isPresent()) {
                    var result = multiTagResult.get();
                    // Multi-tag result gives field-to-camera transform, convert to camera pose then to robot pose
                    Pose3d estimatedRobotPose =
                            new Pose3d().transformBy(result.estimatedPose.best).transformBy(robotToCamera.inverse());
                    posesList.add(estimatedRobotPose);
                    timestampsList.add(res.getTimestampSeconds());
                    ambiguitiesList.add(result.estimatedPose.ambiguity);
                    tagCountsList.add(result.fiducialIDsUsed.size());
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
                        posesList.add(estimatedRobotPose);
                        timestampsList.add(res.getTimestampSeconds());
                        ambiguitiesList.add(target.getPoseAmbiguity());
                        tagCountsList.add(1);
                    }
                }
            }
        }

        // Convert lists to arrays
        inputs.poses = posesList.toArray(new Pose3d[0]);
        inputs.poseTimestamps =
                timestampsList.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.ambiguities =
                ambiguitiesList.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.tagCounts = tagCountsList.stream().mapToInt(Integer::intValue).toArray();

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
        // Set resolution and FOV
        cameraProp.setCalibration(resWidth, resHeight, Rotation2d.fromDegrees(fov));
        // 50 FPS
        cameraProp.setFPS(50);
        // Latency of 35ms with standard deviation of 5ms
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        PhotonCameraSim cameraSim = new PhotonCameraSim(cam, cameraProp);
        visionSim.addCamera(cameraSim, robotToCamera);

        cameraSim.enableDrawWireframe(true);
    }

    @Override
    public void disconnect() {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method disconnect on " + getName(), AlertType.kWarning);
            return;
        }
        disconnected = true;
    }
}
