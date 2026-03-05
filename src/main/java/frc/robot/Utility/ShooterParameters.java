package frc.robot.Utility;

import java.util.Map;

import frc.robot.ScoringLookupTable;

public class ShooterParameters {
    public double hoodAngleDegrees = 17.535;
    public double shooterSpeedRPM = 0;
    public double flightTimeSec = 0;
    public double turretAngle = 0;

    public ShooterParameters(double angle, double speed, double time) {
        hoodAngleDegrees = angle;
        shooterSpeedRPM = speed;
        flightTimeSec = time;
    }

    public ShooterParameters(double angle, double speed, double time, double tangle) {
        hoodAngleDegrees = angle;
        shooterSpeedRPM = speed;
        flightTimeSec = time;
        turretAngle = tangle;
    }

    public double getHoodAngle() {
        return hoodAngleDegrees;
    }

    public double getShooterSpeed() {
        return shooterSpeedRPM;
    }

    public double getTimeOfFlight() {
        return flightTimeSec;
    }

    public double getTurretAngle() {
        return turretAngle;
    }
}