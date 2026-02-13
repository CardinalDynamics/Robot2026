package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.signals.InvertedValue;

public class ShooterConstants {
    public static final int shooterMotorLeftCANID = 52;
    public static final int shooterMotorRightCANID = 53;

    public static final InvertedValue shooterMotorLeftInvertedValue = InvertedValue.Clockwise_Positive;
    public static final InvertedValue shooterMotorRightInvertedValue = InvertedValue.CounterClockwise_Positive;

    // TODO tune these constants
    public static final double kP = .5;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // TODO Find these limits
    public static final double MotionMagicCruiseVelocity = 1000;
    public static final double MotionMagicAcceleration = 1000;

    public static final double shooterIdleRPM = 500;
}
