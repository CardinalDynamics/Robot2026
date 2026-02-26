package frc.robot;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

import frc.robot.Utility.ShooterParameters;

public class ScoringLookupTable {
    private static final NavigableMap<Double, ShooterParameters> table = new TreeMap<>();

    static {
        table.put(1.0, new ShooterParameters(15.0, 1000, 1.0));
        table.put(8.0, new ShooterParameters(  75.0, 4000, 3.0));
    }

    public static ShooterParameters get(double distance) {
        var lower = table.floorEntry(distance);
        var upper = table.ceilingEntry(distance);

        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue();

        double x1 = lower.getKey();
        double x2 = upper.getKey();

        ShooterParameters p1 = lower.getValue();
        ShooterParameters p2 = upper.getValue();

        double t = (distance - x1) / (x2 - x1);

        double degrees = p1.getHoodAngle() + t * (p2.getHoodAngle() - p1.getHoodAngle());
        double rpm = p1.getShooterSpeed() + t * (p2.getShooterSpeed() - p1.getShooterSpeed());
        double sec = p1.getTimeOfFlight() + t * (p2.getTimeOfFlight() - p1.getTimeOfFlight());

        return new ShooterParameters(degrees, rpm, sec);
    }


    public static double velocityToEffectiveDistance(double velocity) {
        // Binary search or iterate through table to find distance
        // where (distance / ToF) = velocity
        // Most InterpolatingTreeMap implementations support inverse lookup
        // or you can build a reverse map: velocity → distance

        for (Map.Entry<Double, ShooterParameters> entry : table.entrySet()) {
            double dist = entry.getKey();
            double vel = dist / entry.getValue().getTimeOfFlight();
            if (vel >= velocity) {
                return dist; // Interpolate for better accuracy
            }
        }
        return table.lastKey(); // Clamp to max
    }
}