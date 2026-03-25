package frc.robot;

import java.nio.file.attribute.PosixFileAttributes;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Utility.ShooterParameters;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.DriveConstants;
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
            ShooterParameters params = getCompensatedAimingParameters();
            if (drivetrain.getAssumedTarget().get() != drivetrain.getHubPose()) {
                params = getCompensatedPassingParameters();
            }
            return params;
        };
    }

    public DoubleSupplier getAssumedTurretAngle() {
        return () -> {
            Double angle = getCompensatedPassingTurret();
            if (drivetrain.getAssumedTarget().get() != drivetrain.getHubPose()) {
                angle = getCompensatedPassingTurret();
            }
            return angle;
        };
    }

    private ShooterParameters getCompensatedAimingParameters() {
        Pose2d estimatedPose = drivetrain.getPose();
        ChassisSpeeds velocity = drivetrain.getRobotRelativeChassisSpeeds();
        estimatedPose = estimatedPose.exp(new Twist2d(
                velocity.vxMetersPerSecond * Constants.latencyCompensation,
                velocity.vyMetersPerSecond * Constants.latencyCompensation,
                velocity.omegaRadiansPerSecond * Constants.latencyCompensation));

        Pose2d turretPose = estimatedPose.transformBy(DriveConstants.shooterOffset);
        Translation2d target = drivetrain.getHubPose().getTranslation();
        double targetDistance = target.getDistance(turretPose.getTranslation());


        Translation2d turretVelocity = new Translation2d(
                        drivetrain.getFieldRelativeChassisSpeeds().vxMetersPerSecond, drivetrain.getFieldRelativeChassisSpeeds().vyMetersPerSecond)
                .plus(new Translation2d(
                        drivetrain.getRobotRelativeChassisSpeeds().omegaRadiansPerSecond
                                * DriveConstants.shooterOffset.getTranslation().getNorm(),
                        estimatedPose.getRotation().rotateBy(Rotation2d.kCW_90deg)));

        double timeOfFlight = ScoringLookupTable.get(targetDistance).getTimeOfFlight();
        Pose2d lookaheadPose = turretPose;
        double lookaheadDistance = targetDistance;

        for (int i = 0; i < 10; i++) {
            double offsetX = turretVelocity.getX() * timeOfFlight;
            double offsetY = turretVelocity.getY() * timeOfFlight;
            lookaheadPose = new Pose2d(
                    turretPose.getTranslation().plus(new Translation2d(offsetX, offsetY)), turretPose.getRotation());
            lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
            timeOfFlight = ScoringLookupTable.get(lookaheadDistance).getTimeOfFlight();
        }


        return ScoringLookupTable.get(lookaheadDistance);
    }

    private double getCompensatedTurret() {
        Pose2d estimatedPose = drivetrain.getPose();
        ChassisSpeeds velocity = drivetrain.getRobotRelativeChassisSpeeds();
        estimatedPose = estimatedPose.exp(new Twist2d(
                velocity.vxMetersPerSecond * Constants.latencyCompensation,
                velocity.vyMetersPerSecond * Constants.latencyCompensation,
                velocity.omegaRadiansPerSecond * Constants.latencyCompensation));

        Pose2d turretPose = estimatedPose.transformBy(DriveConstants.shooterOffset);
        Translation2d target = drivetrain.getHubPose().getTranslation();
        double targetDistance = target.getDistance(turretPose.getTranslation());


        Translation2d turretVelocity = new Translation2d(
                        drivetrain.getFieldRelativeChassisSpeeds().vxMetersPerSecond, drivetrain.getFieldRelativeChassisSpeeds().vyMetersPerSecond)
                .plus(new Translation2d(
                        drivetrain.getRobotRelativeChassisSpeeds().omegaRadiansPerSecond
                                * DriveConstants.shooterOffset.getTranslation().getNorm(),
                        estimatedPose.getRotation().rotateBy(Rotation2d.kCW_90deg)));

        double timeOfFlight = ScoringLookupTable.get(targetDistance).getTimeOfFlight();
        Pose2d lookaheadPose = turretPose;
        double lookaheadDistance = targetDistance;

        for (int i = 0; i < 10; i++) {
            double offsetX = turretVelocity.getX() * timeOfFlight;
            double offsetY = turretVelocity.getY() * timeOfFlight;
            lookaheadPose = new Pose2d(
                    turretPose.getTranslation().plus(new Translation2d(offsetX, offsetY)), turretPose.getRotation());
            lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
            timeOfFlight = ScoringLookupTable.get(lookaheadDistance).getTimeOfFlight();
        }

        final Pose2d finalLookaheadPose = lookaheadPose;
        return turret.getDesiredTurretAngle(() -> target.minus(finalLookaheadPose.getTranslation()).getAngle().getDegrees(), drivetrain::getShooterPose).getAsDouble();
    }

    private ShooterParameters getCompensatedPassingParameters() {
        Pose2d estimatedPose = drivetrain.getPose();
        ChassisSpeeds velocity = drivetrain.getRobotRelativeChassisSpeeds();
        estimatedPose = estimatedPose.exp(new Twist2d(
                velocity.vxMetersPerSecond * Constants.latencyCompensation,
                velocity.vyMetersPerSecond * Constants.latencyCompensation,
                velocity.omegaRadiansPerSecond * Constants.latencyCompensation));

        Pose2d turretPose = estimatedPose.transformBy(DriveConstants.shooterOffset);
        Translation2d target = drivetrain.getAssumedTarget().get().getTranslation();
        double targetDistance = target.getDistance(turretPose.getTranslation());

        Translation2d turretVelocity = new Translation2d(
                        drivetrain.getFieldRelativeChassisSpeeds().vxMetersPerSecond, drivetrain.getFieldRelativeChassisSpeeds().vyMetersPerSecond)
                .plus(new Translation2d(
                        drivetrain.getRobotRelativeChassisSpeeds().omegaRadiansPerSecond
                                * DriveConstants.shooterOffset.getTranslation().getNorm(),
                        estimatedPose.getRotation().rotateBy(Rotation2d.kCW_90deg)));

        double timeOfFlight = PassingLookupTable.get(targetDistance).getTimeOfFlight();
        Pose2d lookaheadPose = turretPose;
        double lookaheadDistance = targetDistance;

        for (int i = 0; i < 10; i++) {
            double offsetX = turretVelocity.getX() * timeOfFlight;
            double offsetY = turretVelocity.getY() * timeOfFlight;
            lookaheadPose = new Pose2d(
                    turretPose.getTranslation().plus(new Translation2d(offsetX, offsetY)), turretPose.getRotation());
            lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
            timeOfFlight = PassingLookupTable.get(lookaheadDistance).getTimeOfFlight();
        }


        return PassingLookupTable.get(lookaheadDistance);
    }

    private double getCompensatedPassingTurret() {
        Pose2d estimatedPose = drivetrain.getPose();
        ChassisSpeeds velocity = drivetrain.getRobotRelativeChassisSpeeds();
        estimatedPose = estimatedPose.exp(new Twist2d(
                velocity.vxMetersPerSecond * Constants.latencyCompensation,
                velocity.vyMetersPerSecond * Constants.latencyCompensation,
                velocity.omegaRadiansPerSecond * Constants.latencyCompensation));

        Pose2d turretPose = estimatedPose.transformBy(DriveConstants.shooterOffset);
        Translation2d target = drivetrain.getAssumedTarget().get().getTranslation();
        double targetDistance = target.getDistance(turretPose.getTranslation());

        Translation2d turretVelocity = new Translation2d(
                        drivetrain.getFieldRelativeChassisSpeeds().vxMetersPerSecond, drivetrain.getFieldRelativeChassisSpeeds().vyMetersPerSecond)
                .plus(new Translation2d(
                        drivetrain.getRobotRelativeChassisSpeeds().omegaRadiansPerSecond
                                * DriveConstants.shooterOffset.getTranslation().getNorm(),
                        estimatedPose.getRotation().rotateBy(Rotation2d.kCW_90deg)));

        double timeOfFlight = PassingLookupTable.get(targetDistance).getTimeOfFlight();
        Pose2d lookaheadPose = turretPose;
        double lookaheadDistance = targetDistance;

        for (int i = 0; i < 10; i++) {
            double offsetX = turretVelocity.getX() * timeOfFlight;
            double offsetY = turretVelocity.getY() * timeOfFlight;
            lookaheadPose = new Pose2d(
                    turretPose.getTranslation().plus(new Translation2d(offsetX, offsetY)), turretPose.getRotation());
            lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
            timeOfFlight = PassingLookupTable.get(lookaheadDistance).getTimeOfFlight();
        }

        final Pose2d finalLookaheadPose = lookaheadPose;
        return turret.getDesiredTurretAngle(() -> target.minus(finalLookaheadPose.getTranslation()).getAngle().getDegrees(), drivetrain::getShooterPose).getAsDouble();
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
            turret.getTurretPIDCommand(() -> getCompensatedTurret()),
            hood.getHoodPIDCommand(() -> getCompensatedAimingParameters().hoodAngleDegrees),
            shooter.getShooterPIDCommand(() -> getCompensatedAimingParameters().shooterSpeedRPM)
        );
    }
}