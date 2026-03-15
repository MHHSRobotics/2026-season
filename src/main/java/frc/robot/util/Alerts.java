package frc.robot.util;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

// Utility class for one-time alerts
public class Alerts {
    private Set<String> dedup = new HashSet<>();
    private List<Alert> alerts = new ArrayList<>();
    private static Alerts alertsObj;

    public static Alerts getInstance() {
        if (alertsObj == null) {
            alertsObj = new Alerts();
        }
        return alertsObj;
    }

    public void createAlert(String text, AlertType type) {
        if (!dedup.contains(text)) {
            Alert alert = new Alert(text, type);
            alert.set(true);
            alerts.add(alert);
            dedup.add(text);
        }
    }

    // Create a permanent alert with the given text and AlertType (info, warning, or error)
    public static void create(String text, AlertType type) {
        getInstance().createAlert(text, type);
    }
}
