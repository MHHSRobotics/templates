package frc.robot.io;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;

public class EncoderIO {
    @AutoLog
    public static class EncoderIOInputs {
        public boolean connected;

        public double positionRad; // mechanism radians
        public double velocityRadPerSec; // mechanism radians per sec

        public boolean badMagnetFault;
        public boolean hardwareFault;
    }

    private String logPath;

    private String name;

    // Alert objects to show encoder problems on the dashboard
    private Alert disconnectAlert;
    private Alert hardwareFaultAlert;
    private Alert magnetFaultAlert;

    public EncoderIO(String name, String logPath) {
        this.name = name;
        this.logPath = logPath;

        // Create alerts with descriptive names for this encoder
        disconnectAlert = new Alert("The " + name + " is disconnected", AlertType.kError);
        hardwareFaultAlert = new Alert("The " + name + " encountered an internal hardware fault", AlertType.kError);
        magnetFaultAlert = new Alert("The " + name + " magnet is not functioning", AlertType.kError);
    }

    public String getName() {
        return name;
    }

    protected EncoderIOInputsAutoLogged inputs = new EncoderIOInputsAutoLogged();

    // Find out the latest values from the encoder and store them in inputs
    public void update() {
        // Log the inputs to AdvantageKit
        Logger.processInputs(logPath, inputs);

        // Update alerts based on the current encoder status (this runs after subclass updates inputs)
        disconnectAlert.set(!inputs.connected);
        hardwareFaultAlert.set(inputs.hardwareFault);
        magnetFaultAlert.set(inputs.badMagnetFault);
    }

    public EncoderIOInputs getInputs() {
        return inputs;
    }

    private void unsupportedFeature() {
        if (Constants.currentMode != Mode.REPLAY) {
            Alerts.create("An unsupported feature was used on " + getName(), AlertType.kWarning);
        }
    }

    public void setGearRatio(double ratio) {
        unsupportedFeature();
    }

    public void setOffset(double offset) {
        unsupportedFeature();
    }

    public void setPosition(double position) {
        unsupportedFeature();
    }

    public void setInverted(boolean inverted) {
        unsupportedFeature();
    }

    public void setMechPosition(double position) {
        unsupportedFeature();
    }

    public void setMechVelocity(double velocity) {
        unsupportedFeature();
    }

    // Disconnect the encoder. Simulation-only.
    public void disconnect() {
        unsupportedFeature();
    }
}
