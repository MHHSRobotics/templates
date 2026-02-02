package frc.robot.io;

import edu.wpi.first.wpilibj.Alert.AlertType;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;

public class BitIO {
    @AutoLog
    public static class BitIOInputs {
        public boolean value;
    }

    private String name;
    private String logPath;

    public BitIO(String name, String logPath) {
        this.name = name;
        this.logPath = logPath;
    }

    public String getName() {
        return name;
    }

    protected BitIOInputsAutoLogged inputs = new BitIOInputsAutoLogged();

    public void update() {
        // Log the inputs to AdvantageKit
        Logger.processInputs(logPath, inputs);
    }

    public BitIOInputs getInputs() {
        return inputs;
    }

    private void unsupportedFeature() {
        if (Constants.currentMode != Mode.REPLAY) {
            Alerts.create("An unsupported feature was used on " + getName(), AlertType.kWarning);
        }
    }

    // Sim-only, sets the value of the simulated BitIO
    public void setValue(boolean value) {
        unsupportedFeature();
    }
}
