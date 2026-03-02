package frc.robot.subsystems.Hood;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class HoodConstants {

    public static final int hoodMotorCANID = 52;

    public static final double gearRatio = 76.533;

    public static final double kP = 3.0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double MotionMagicCruiseVelocity = 2067;
    public static final double MotionMagicAcceleration = 2067;

    public static final double hoodOffset = 17.535;

    public static final double hoodMaxLimit = 43.0;
    public static final double hoodMinLimit = 17.535;

    public static final double hoodStowSetpoint = 20.0;

    public static final double hoodTolerance = .5;
}
