package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Utility.ShooterParameters;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Indexer.KickerSubsystem;
import frc.robot.subsystems.Indexer.SpindexerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;

public class AutoAim {
    CommandSwerveDrivetrain drivetrain;
    TurretSubsystem turret;
    HoodSubsystem hood;
    ShooterSubsystem shooter;
    SpindexerSubsystem spindexer;
    KickerSubsystem kicker;

    public AutoAim(CommandSwerveDrivetrain drivetrain, TurretSubsystem turret, HoodSubsystem hood, ShooterSubsystem shooter, SpindexerSubsystem spindexer, KickerSubsystem kicker) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.kicker = kicker;
    }

    public BooleanSupplier isReadyToShoot(BooleanSupplier a, BooleanSupplier b, BooleanSupplier c) {
        return () -> {
            return a.getAsBoolean() && b.getAsBoolean() && c.getAsBoolean();
        };
    }

    public DoubleSupplier getSpindexerSpeed(BooleanSupplier a, BooleanSupplier b, BooleanSupplier c) {
        return () -> {
            double rpm = 0;
            if (isReadyToShoot(a, b, c).getAsBoolean()) {
                rpm = IndexerConstants.SpindexSpeed;
            }
            return rpm;
        };
    }

    public Supplier<ShooterParameters> getScoringShooterParamaters() {
        return () -> {
            ShooterParameters params = ScoringLookupTable.get(drivetrain.getShooterPose().getTranslation()
                .getDistance(drivetrain.getHubPose().getTranslation()));
            return params;
        };
    }

    public Supplier<ShooterParameters> getPassingShooterParamaters(Supplier<Pose2d> drivePose) {
        return () -> {
            ShooterParameters params = PassingLookupTable.get(drivePose.get().getTranslation()
                .getDistance(drivetrain.getLeftPassingPose().getTranslation()));
            return params;
        };
    }

    public Supplier<ShooterParameters> getAssumedShooterParamaters() {
        return () -> {
            ShooterParameters params = ScoringLookupTable.get(drivetrain.getShooterPose().getTranslation()
                .getDistance(drivetrain.getHubPose().getTranslation()));
            if (drivetrain.getAssumedTarget().get() != drivetrain.getHubPose()) {
                params = getPassingSOTMParams().get();
            }
            return params;
        };
    }

    public Supplier<ShooterParameters> getScoringSOTMParams() {
        return () -> {
            Translation2d futurePos = drivetrain.getShooterPose().getTranslation().plus(drivetrain.getRobotVelocityVector().times(Constants.latencyCompensation));
            Translation2d toGoal = drivetrain.getHubPose().getTranslation().minus(futurePos);
            double distance = toGoal.getNorm();
            Translation2d targetDirection = toGoal.div(distance);
            ShooterParameters baseline = ScoringLookupTable.get(distance);
            double baselineVelocity = distance / baseline.getTimeOfFlight();
            Translation2d targetVelocity = targetDirection.times(baselineVelocity);
            Translation2d shotVelocity = targetVelocity.minus(drivetrain.getRobotVelocityVector());
            double turretAngle = turret.getDesiredTurretAngle(() -> shotVelocity.getAngle().getDegrees(), drivetrain::getShooterPose).getAsDouble();
            double requiredVelocity = shotVelocity.getNorm();
            double effectiveDistance = ScoringLookupTable.velocityToEffectiveDistance(requiredVelocity);
            ShooterParameters newParams = ScoringLookupTable.get(effectiveDistance);
            return newParams;
        };
    }

    public Supplier<ShooterParameters> getPassingSOTMParams() {
        return () -> {
            Translation2d futurePos = drivetrain.getShooterPose().getTranslation().plus(drivetrain.getRobotVelocityVector().times(Constants.latencyCompensation));
            Translation2d toGoal = drivetrain.getAssumedTarget().get().getTranslation().minus(futurePos);
            double distance = toGoal.getNorm();
            Translation2d targetDirection = toGoal.div(distance);
            ShooterParameters baseline = PassingLookupTable.get(distance);
            double baselineVelocity = distance / baseline.getTimeOfFlight();
            Translation2d targetVelocity = targetDirection.times(baselineVelocity);
            Translation2d shotVelocity = targetVelocity.minus(drivetrain.getRobotVelocityVector());
            double turretAngle = turret.getDesiredTurretAngle(() -> shotVelocity.getAngle().getDegrees(), drivetrain::getShooterPose).getAsDouble();
            double requiredVelocity = shotVelocity.getNorm();
            double effectiveDistance = PassingLookupTable.velocityToEffectiveDistance(requiredVelocity);
            ShooterParameters newParams = PassingLookupTable.get(effectiveDistance);
            return newParams;
        };
    }

    public Supplier<Double> getScoringSOTMTurret() {
        return () -> {
            Translation2d futurePos = drivetrain.getShooterPose().getTranslation().plus(drivetrain.getRobotVelocityVector().times(Constants.latencyCompensation));
            Translation2d toGoal = drivetrain.getHubPose().getTranslation().minus(futurePos);
            double distance = toGoal.getNorm();
            Translation2d targetDirection = toGoal.div(distance);
            ShooterParameters baseline = ScoringLookupTable.get(distance);
            double baselineVelocity = distance / baseline.getTimeOfFlight();
            Translation2d targetVelocity = targetDirection.times(baselineVelocity);
            Translation2d shotVelocity = targetVelocity.minus(drivetrain.getRobotVelocityVector());
            double turretAngle = turret.getDesiredTurretAngle(() -> shotVelocity.getAngle().getDegrees(), drivetrain::getShooterPose).getAsDouble();
            return turretAngle;
        };
    }

    public Supplier<Double> getPassingSOTMTurret() {
        return () -> {
            Translation2d futurePos = drivetrain.getShooterPose().getTranslation().plus(drivetrain.getRobotVelocityVector().times(Constants.latencyCompensation));
            Translation2d toGoal = drivetrain.getAssumedTarget().get().getTranslation().minus(futurePos);
            double distance = toGoal.getNorm();
            Translation2d targetDirection = toGoal.div(distance);
            ShooterParameters baseline = PassingLookupTable.get(distance);
            double baselineVelocity = distance / baseline.getTimeOfFlight();
            Translation2d targetVelocity = targetDirection.times(baselineVelocity);
            Translation2d shotVelocity = targetVelocity.minus(drivetrain.getRobotVelocityVector());
            double turretAngle = turret.getDesiredTurretAngle(() -> shotVelocity.getAngle().getDegrees(), drivetrain::getShooterPose).getAsDouble();
            return turretAngle;
        };
    }

    public DoubleSupplier getAssumedTurretAngle() {
        return () -> {
            Double angle = turret.getDesiredTurretAngle(drivetrain::getHubPose, drivetrain::getShooterPose).getAsDouble();
            if (drivetrain.getAssumedTarget().get() != drivetrain.getHubPose()) {
                angle = getPassingSOTMTurret().get();
            }
            return angle;
        };
    }

    public Command generateTurretScoreCommand() {
        return turret.getTurretPIDCommand(getAssumedTurretAngle());
    }

    public Command generateTurretIdleCommand() {
        return turret.getTurretPIDCommand(getAssumedTurretAngle());
    }

    public Command generateHoodScoreCommand() {
        return hood.getHoodPIDCommand(getScoringShooterParamaters());
    }

    public Command generateHoodIdleCommand() {
        return hood.getHoodPIDCommand(getAssumedShooterParamaters());
    }

    public Command generateAssumedShooterCommand() {
        return shooter.getShooterPIDCommand(getAssumedShooterParamaters());
    }
    
    public Command generateSOTMScoringCommand() {
        return new ParallelCommandGroup(
            turret.getTurretPIDCommand(() -> getScoringSOTMTurret().get()),
            hood.getHoodPIDCommand(() -> getScoringSOTMParams().get().hoodAngleDegrees),
            shooter.getShooterPIDCommand(() -> getScoringSOTMParams().get().shooterSpeedRPM)
        );
    }
}