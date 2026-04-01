package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final CANBus canivoreBus = new CANBus("can_1");

    public static final double horizontalCenterLine = Units.inchesToMeters(317.69 / 2.0);
    public static final Pose2d blueHubPose = new Pose2d(Units.inchesToMeters(182.11), horizontalCenterLine, new Rotation2d());
    public static final Pose2d blueLeftPassTarget = new Pose2d(1.1, 6, new Rotation2d());
    public static final Pose2d blueRightPassTarget = new Pose2d(1.1, 2.1, new Rotation2d());
    public static final Pose2d redHubPose = new Pose2d(Units.inchesToMeters(651.22 - 182.11), horizontalCenterLine, new Rotation2d());
    public static final Pose2d redLeftPassTarget = new Pose2d(15.4, 2.1, new Rotation2d());
    public static final Pose2d redRightPassTarget = new Pose2d(15.4, 6, new Rotation2d());

    // TODO tune this and get for both alliances
    public static final double blueAllianceZoneCutoffMeters = Units.inchesToMeters(182.11);
    public static final double redAllianceZoneCutoffMeters = Units.inchesToMeters(651.22 - 182.11);

    public static final double latencyCompensation = .03;
}
