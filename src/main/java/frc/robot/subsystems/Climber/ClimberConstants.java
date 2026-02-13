package frc.robot.subsystems.Climber;

public class ClimberConstants {
    public static final double gearRatio = 1.0;

    public static final int climbMotorCANID = 55;
    
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

    // TODO find these
    public static final double climbedPosition = 0;
    public static final double deployedPosition = 10;
}
