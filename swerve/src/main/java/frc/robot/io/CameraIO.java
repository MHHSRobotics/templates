package frc.robot.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;
import org.photonvision.simulation.VisionSystemSim;

public class CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public boolean connected;

        // Camera pose data as dynamic arrays (sized to actual measurements)
        public Pose3d[] poses = new Pose3d[0];
        public double[] poseTimestamps = new double[0]; // Timestamp of each pose (seconds)
        public double[] ambiguities = new double[0]; // Pose ambiguity for each measurement
        public int[] tagCounts = new int[0]; // Number of tags in each measurement
    }

    private String name;
    private String logPath;

    private Alert disconnectAlert;

    public CameraIO(String name, String logPath) {
        this.name = name;
        this.logPath = logPath;

        // Create alerts with descriptive names for this camera
        disconnectAlert = new Alert("The " + name + " is disconnected", AlertType.kError);
    }

    public String getName() {
        return name;
    }

    protected CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

    public void update() {
        // Log the inputs to AdvantageKit
        Logger.processInputs(logPath, inputs);

        // Update alerts based on the current encoder status (this runs after subclass updates inputs)
        disconnectAlert.set(!inputs.connected);
    }

    public CameraIOInputs getInputs() {
        return inputs;
    }

    private void unsupportedFeature() {
        if (Constants.currentMode != Mode.REPLAY) {
            Alerts.create("An unsupported feature was used on " + getName(), AlertType.kWarning);
        }
    }

    public void startSim(VisionSystemSim sim) {
        unsupportedFeature();
    }

    // Disconnect the camera, simulation-only.
    public void disconnect() {
        unsupportedFeature();
    }
}
