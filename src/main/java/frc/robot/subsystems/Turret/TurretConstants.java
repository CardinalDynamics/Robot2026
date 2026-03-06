package frc.robot.subsystems.Turret;

public final class TurretConstants {

    public static final int turretMotorCANID = 51;

    public static final double kP = 10;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // was 200
    public static final double MotionMagicCruiseVelocity = 200;
    public static final double MotionMagicAcceleration = 200;
    public static final double gearRatio = 37.9116666;

    // TODO find these limits
    public static final double clockwiseTurretLimitDegrees = 130;
    public static final double counterclockwiseTurretLimitDegrees = -270;

    // TODO find this offset
    public static final double turretOffset = -90.0;

    public static final double turretTolerance = 1.0;
}