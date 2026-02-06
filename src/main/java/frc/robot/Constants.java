package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
    
    public static final CANBus canivoreBus = new CANBus("can_1");

    // TODO further tune this and get for both alliances
    public static final Pose2d hubPose = new Pose2d(4.5, 3.9, new Rotation2d());

}
