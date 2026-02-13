package frc.robot;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Utility.ShooterParameters;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;

public class AutoAim {

    CommandSwerveDrivetrain drivetrain;
    TurretSubsystem turret;
    HoodSubsystem hood;
    ShooterSubsystem shooter;

    public AutoAim(CommandSwerveDrivetrain drivetrain, TurretSubsystem turret, HoodSubsystem hood, ShooterSubsystem shooter) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
    }

    public Supplier<ShooterParameters> getScoringShooterParamaters(Supplier<Pose2d> drivePose) {
        return () -> {
            ShooterParameters params = ScoringLookupTable.get(drivePose.get().getTranslation()
                .getDistance(Constants.hubPose.getTranslation()));
            return params;
        };
    }

    public Supplier<ShooterParameters> getPassingShooterParamaters(Supplier<Pose2d> drivePose) {
        return () -> {
            ShooterParameters params = PassingLookupTable.get(drivePose.get().getTranslation()
                .getDistance(Constants.hubPose.getTranslation()));
            return params;
        };
    }

    public Supplier<ShooterParameters> getAssumedShooterParamaters(Supplier<Pose2d> drivePose) {
        return () -> {
            ShooterParameters params = ScoringLookupTable.get(drivePose.get().getTranslation()
                .getDistance(Constants.hubPose.getTranslation()));
            if (drivetrain.getAssumedTarget().get() == Constants.passTarget) {
                params = PassingLookupTable.get(drivePose.get().getTranslation()
                .getDistance(Constants.hubPose.getTranslation()));
            }
            return params;
        };
    }


    public Command generateTurretScoreCommand() {
        return Commands.defer(() -> turret.getTurretPIDCommand(turret.getDesiredTurretAngle(() -> Constants.hubPose, drivetrain::getShooterPose)), Set.of(turret));
    }

    public Command generateTurretIdleCommand() {
        return Commands.defer(() -> turret.getTurretPIDCommand(turret.getDesiredTurretAngle(drivetrain.getAssumedTarget(), drivetrain::getShooterPose)), Set.of(turret));
    }

    public Command generateHoodScoreCommand() {
        return Commands.defer(() -> hood.getHoodPIDCommand(getScoringShooterParamaters(drivetrain::getShooterPose)), Set.of(hood));
    }

    public Command generateHoodIdleCommand() {
        return Commands.defer(() -> hood.getHoodPIDCommand(getAssumedShooterParamaters(drivetrain::getShooterPose)), Set.of(hood));
    }

    public Command generateAssumedShooterCommand() {
        return Commands.defer(() -> shooter.getShooterPIDCommand(getAssumedShooterParamaters(drivetrain::getShooterPose)), Set.of(shooter));
    }
    
}