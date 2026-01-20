package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

// Utility class for one-time alerts
public class Alerts {
    private List<Alert> alerts = new ArrayList<>();
    private static Alerts alertsObj;

    public static Alerts getInstance() {
        if (alertsObj == null) {
            alertsObj = new Alerts();
        }
        return alertsObj;
    }

    public void createAlert(String text, AlertType type) {
        Alert alert = new Alert(text, type);
        alert.set(true);
        alerts.add(alert);
    }

    public static void create(String text, AlertType type) {
        getInstance().createAlert(text, type);
    }
}
