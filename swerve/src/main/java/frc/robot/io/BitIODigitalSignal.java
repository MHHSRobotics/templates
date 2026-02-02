package frc.robot.io;

import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;

public class BitIODigitalSignal extends BitIO {
    private DigitalInput input;

    private int id;
    private boolean simValue;

    public BitIODigitalSignal(String name, String logPath, int id) {
        super(name, logPath);
        input = new DigitalInput(id);
        this.id = id;
    }

    public int getId() {
        return id;
    }

    @Override
    public void update() {
        inputs.value = Constants.currentMode == Mode.REAL ? input.get() : simValue;
    }

    @Override
    public void setValue(boolean value) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setValue on " + getName(), AlertType.kWarning);
            return;
        }
        simValue = value;
    }
}
