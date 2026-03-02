package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.signals.InvertedValue;

public class ShooterConstants {
    public static final int shooterMotorLeftCANID = 53;
    public static final int shooterMotorRightCANID = 54;

    public static final InvertedValue shooterMotorLeftInvertedValue = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue shooterMotorRightInvertedValue = InvertedValue.Clockwise_Positive;

    public static final double kP = .35;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = .118;
    public static final double kA = 0;

    // TODO Find these limits
    public static final double MotionMagicCruiseVelocity = 300;
    public static final double MotionMagicAcceleration = 300;

    public static final double shooterIdleRPM = 500;

    public static final double shooterToleranceRPM = 100;
}
