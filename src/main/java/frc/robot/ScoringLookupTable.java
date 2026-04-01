package frc.robot;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utility.ShooterParameters;

public class ScoringLookupTable {
    private static final NavigableMap<Double, ShooterParameters> table = new TreeMap<>();

    static {
        // table.put(2.3087, new ShooterParameters(18.0, 2950, 1.1));
        // table.put(2.843, new ShooterParameters(18.721, 3300, 1.35));
        // table.put(4.1368, new ShooterParameters(20, 3700, 1.5));
        // table.put(1.7113, new ShooterParameters(18.0, 2500, .86));
        // table.put(2.443, new ShooterParameters(20.0, 2900, 1.166));
        // table.put(3.65, new ShooterParameters(24.0, 3300, 1.35));
        // table.put(4.24, new ShooterParameters(27.0, 3500, 1.266));
        // table.put(5.479, new ShooterParameters(30.0, 3800, 1.4));
        // table.put(6.59, new ShooterParameters(34.0, 4200, 1.266));
        // table.put(9.18, new ShooterParameters(36.0, 5950, 1.4));
        table.put(1.7113, new ShooterParameters(18.0, 2500, .86));
        table.put(2.443, new ShooterParameters(20.0, 2900, 1.166));
        table.put(3.65, new ShooterParameters(24.0, 3200, 1.35));
        table.put(4.24, new ShooterParameters(27.0, 3400, 1.266));
        table.put(5.479, new ShooterParameters(30.0, 3700, 1.4));
        table.put(6.59, new ShooterParameters(34.0, 4100, 1.266));
        table.put(9.18, new ShooterParameters(36.0, 5850, 1.4));
    }

    public static ShooterParameters get(double distance) {
        double clampedDist = MathUtil.clamp(distance, 1.7113, 9.18);
        var lower = table.floorEntry(clampedDist);
        var upper = table.ceilingEntry(clampedDist);

        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue();

        double x1 = lower.getKey();
        double x2 = upper.getKey();

        if (x1 == x2) {
            return lower.getValue();
        }

        ShooterParameters p1 = lower.getValue();
        ShooterParameters p2 = upper.getValue();

        double t = (clampedDist - x1) / (x2 - x1);

        double degrees = p1.getHoodAngle() + t * (p2.getHoodAngle() - p1.getHoodAngle());
        double rpm = p1.getShooterSpeed() + t * (p2.getShooterSpeed() - p1.getShooterSpeed());
        double sec = p1.getTimeOfFlight() + t * (p2.getTimeOfFlight() - p1.getTimeOfFlight());

        return new ShooterParameters(degrees, rpm, sec);
    }


    public static double velocityToEffectiveDistance(double velocity) {

        Map.Entry<Double, ShooterParameters> prev = null;

        for (Map.Entry<Double, ShooterParameters> entry : table.entrySet()) {

            double dist = entry.getKey();
            double vel = dist / entry.getValue().getTimeOfFlight();

            if (prev != null) {
                double prevDist = prev.getKey();
                double prevVel = prevDist / prev.getValue().getTimeOfFlight();

                if (vel >= velocity) {

                    // ----- interpolation -----
                    double t = (velocity - prevVel) / (vel - prevVel);
                    t = Math.max(0, Math.min(1, t)); // clamp
                    return prevDist + t * (dist - prevDist);
                }
            }

            prev = entry;
        }

        return table.lastKey(); // clamp to max
    }
}