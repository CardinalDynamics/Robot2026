package frc.robot;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utility.ShooterParameters;

public class PassingLookupTable {
    private static final NavigableMap<Double, ShooterParameters> table = new TreeMap<>();

    static {
        table.put(.5, new ShooterParameters(18.0, 1600, 1.0));
        table.put(5.0, new ShooterParameters(30.0, 2300, 1.5));
        table.put(9.0, new ShooterParameters(36.0, 4000, 2.0));
        table.put(13.0, new ShooterParameters(36.0, 6000, 2.5));
    }

    public static ShooterParameters get(double distance) {
        double clampedDist = MathUtil.clamp(distance, .5, 13.0);
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