package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
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

    public Supplier<ShooterParameters> getScoringShooterParamaters(Supplier<Pose2d> drivePose) {
        return () -> {
            ShooterParameters params = ScoringLookupTable.get(drivePose.get().getTranslation()
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

    public Supplier<ShooterParameters> getAssumedShooterParamaters(Supplier<Pose2d> drivePose) {
        return () -> {
            ShooterParameters params = ScoringLookupTable.get(drivePose.get().getTranslation()
                .getDistance(drivetrain.getHubPose().getTranslation()));
            if (drivetrain.getAssumedTarget().get() != drivetrain.getHubPose()) {
                params = PassingLookupTable.get(drivePose.get().getTranslation()
                .getDistance(drivetrain.getAssumedTarget().get().getTranslation()));
            }
            return params;
        };
    }

    public Supplier<ShooterParameters> getScoringSOTMCommand(Supplier<Pose2d> drivePose) {
        return () -> {
            Translation2d futurePos = drivePose.get().getTranslation().plus(drivetrain.getRobotVelocityVector().times(Constants.latencyCompensation));
            Translation2d toGoal = drivetrain.getHubPose().getTranslation().minus(futurePos);
            double distance = toGoal.getNorm();
            Translation2d targetDirection = toGoal.div(distance);
            ShooterParameters baseline = ScoringLookupTable.get(distance);
            double baselineVelocity = distance / baseline.getTimeOfFlight();
            Translation2d targetVelocity = targetDirection.times(baselineVelocity);
            Translation2d shotVelocity = targetVelocity.minus(drivetrain.getRobotVelocityVector());
            Rotation2d turretAngle = shotVelocity.getAngle();
            double requiredVelocity = shotVelocity.getNorm();
            double effectiveDistance = ScoringLookupTable.velocityToEffectiveDistance(requiredVelocity);
            ShooterParameters newParams = ScoringLookupTable.get(effectiveDistance);
            return new ShooterParameters(newParams.getHoodAngle(), newParams.getShooterSpeed(), turretAngle.getDegrees());
        };
    }


    public Command generateTurretScoreCommand() {
        return Commands.defer(() -> turret.getTurretPIDCommand(turret.getDesiredTurretAngle(drivetrain::getHubPose, drivetrain::getShooterPose)), Set.of(turret));
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
    
    public Command generateSpindexerCommand() {
        return Commands.defer(() -> spindexer.getSpindexerCommand(getSpindexerSpeed(hood::getHoodAtPosition, hood::getHoodAtPosition, shooter::getShooterAtSpeed)), Set.of(spindexer));
    }

    public Command generateSOTMScoringCommand() {
        return new ParallelCommandGroup();
    }
}