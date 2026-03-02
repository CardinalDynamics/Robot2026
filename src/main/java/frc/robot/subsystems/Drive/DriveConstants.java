package frc.robot.subsystems.Drive;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;


public class DriveConstants {

    // TODO find this
    public static final Transform2d shooterOffset = new Transform2d(new Translation2d(Units.inchesToMeters(4.453), -Units.inchesToMeters(5.875)), new Rotation2d());

    public static final Transform3d ROBOT_TO_QUEST = new Transform3d( /*TODO: Put your x, y, z, yaw, pitch, and roll offsets here!*/ );

    public static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
    VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );

    public static final Matrix<N3, N1> LIMELIGHT_STD_DEVS =
    VecBuilder.fill(0.7, 0.7, 9999999);

    public static final PPHolonomicDriveController ppController =
    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    );
}
