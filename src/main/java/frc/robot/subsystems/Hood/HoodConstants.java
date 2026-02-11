package frc.robot.subsystems.Hood;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class HoodConstants {

    public static final int hoodMotorCANID = 52;

    public static final double gearRatio = 1;

    // TODO tune these constants
    public static final double kP = .5;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // TODO Find these limits
    public static final double MotionMagicCruiseVelocity = 10;
    public static final double MotionMagicAcceleration = 10;

    // TODO find this offset
    public static final double hoodOffset = 15;

    // TODO find these limits
    public static final double hoodMaxLimit = 72;
    public static final double hoodMinLimit = 15;
}
