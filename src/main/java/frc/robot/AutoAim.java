package frc.robot;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.TurretSubsystem;

public class AutoAim {

    CommandSwerveDrivetrain drivetrain;
    TurretSubsystem turret;

    public AutoAim(CommandSwerveDrivetrain drivetrain, TurretSubsystem turret) {
        this.drivetrain = drivetrain;
        this.turret = turret;
    }

    public Command generateTurretCommand() {
        return Commands.defer(() -> turret.getTurretPIDCommand(turret.getDesiredTurretAngle(() -> Constants.hubPose, drivetrain::getPose)), Set.of(turret));
    }

    public Command generateTurretIdleCommand() {
        return Commands.defer(() -> turret.getTurretPIDCommand(turret.getDesiredTurretAngle(drivetrain.getAssumedTarget(), drivetrain::getPose)), Set.of(turret));
    }
    
}