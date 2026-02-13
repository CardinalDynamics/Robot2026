package frc.robot.Utility;

public class ShooterParameters {
    public double hoodAngleDegrees;
    public double shooterSpeedRPM;
    public double flightTimeSec;

    public ShooterParameters(double angle, double speed, double time) {
        hoodAngleDegrees = angle;
        shooterSpeedRPM = speed;
        flightTimeSec = time;
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
}