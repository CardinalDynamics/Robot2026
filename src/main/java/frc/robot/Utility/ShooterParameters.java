package frc.robot.Utility;

import java.util.Map;

import frc.robot.ScoringLookupTable;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterParameters {
    public double hoodAngleDegrees = 17.535;
    public double shooterSpeedRPM = 0;
    public double flightTimeSec = 0;
    public double turretAngle = 0;
    public double turretRadPerSec = 0;

    public ShooterParameters(double angle, double speed, double time) {
        hoodAngleDegrees = angle;
        shooterSpeedRPM = speed;
        flightTimeSec = time;
    }

    public ShooterParameters(double angle, double speed, double time, double turretAngle, double turretRadPerSec) {
        hoodAngleDegrees = angle;
        shooterSpeedRPM = speed;
        flightTimeSec = time;
        this.turretAngle = turretAngle;
        this.turretRadPerSec = turretRadPerSec;
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

    public double getRotationalRate() {
        return turretRadPerSec;
    }

    public ShooterParameters withturretAngle(double turretAngle) {
        this.turretAngle = turretAngle;
        return this;
    }

    public ShooterParameters withRotationalRate(double turretRadPerSec) {
        this.turretRadPerSec = turretRadPerSec;
        return this;
    }
}