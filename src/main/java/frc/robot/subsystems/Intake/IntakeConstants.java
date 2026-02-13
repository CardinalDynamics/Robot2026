package frc.robot.subsystems.Intake;

public class IntakeConstants {
    public static final int pivotMotorCANDID = 55;
    public static final int wheelMotorCANID = 61;

    public static final double pivotGearRatio = 1;
    public static final double wheelGearRatio = 1;

    // TODO tune these constants
    public static final double kPivotP = .5;
    public static final double kPivotI = 0;
    public static final double kPivotD = 0;
    public static final double kPivotS = 0;
    public static final double kPivotV = 0;
    public static final double kPivotA = 0;

    // TODO Find these limits
    public static final double PivotMotionMagicCruiseVelocity = 10;
    public static final double PivotMotionMagicAcceleration = 10;

    public static final double pivotOffset = 0;

    public static final double pivotStowPosiion = 0;
    public static final double pivotDeployPosition = 90;

    public static final double kWheelP = .5;
    public static final double kWheelI = 0;
    public static final double kWheelD = 0;
    public static final double IntakeWheelMaxAcceleration = 10;
    public static final double IntakeWheelMaxVelocity = 10;

    public static final double IntakeSpeed = 1000;
    
}
