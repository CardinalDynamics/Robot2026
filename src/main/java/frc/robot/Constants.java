package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public final class Constants {
    
    public static final CANBus canivoreBus = new CANBus("can_1");

    // TODO further tune this and get for both alliances
    public static final Pose2d blueHubPose = new Pose2d(4.5, 3.9, new Rotation2d());
    public static final Pose2d blueLeftPassTarget = new Pose2d(.1, 8, new Rotation2d());
    public static final Pose2d blueRightPassTarget = new Pose2d(.1, .1, new Rotation2d());
    public static final Pose2d redHubPose = new Pose2d(4.5, 3.9, new Rotation2d());
    public static final Pose2d redLeftPassTarget = new Pose2d(16.4, 0.1, new Rotation2d());
    public static final Pose2d redRightPassTarget = new Pose2d(16.4, 8, new Rotation2d());

    // TODO tune this and get for both alliances
    public static final double blueAllianceZoneCutoffMeters = 4.5;
    public static final double redAllianceZoneCutoffMeters = 12.0;
    public static final double verticalHalfMeters = 4.0;

    public static final double latencyCompensation = .1;
}
