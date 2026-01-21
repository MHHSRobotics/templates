package frc.robot.util;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

// Utility class for one-time alerts
public class Alerts {
    // Checks for duplicate alert texts so alerts don't spam and crash networktables
    private HashSet<String> prevAlerts = new HashSet<>();
    private List<Alert> alerts = new ArrayList<>();
    private static Alerts alertsObj;

    public static Alerts getInstance() {
        if (alertsObj == null) {
            alertsObj = new Alerts();
        }
        return alertsObj;
    }

    public void createAlert(String text, AlertType type) {
        if (prevAlerts.contains(text)) {
            return;
        }
        Alert alert = new Alert(text, type);
        alert.set(true);
        alerts.add(alert);
        prevAlerts.add(text);
    }

    // Create a permanent alert with the given text and AlertType (info, warning, or error)
    public static void create(String text, AlertType type) {
        getInstance().createAlert(text, type);
    }
}
