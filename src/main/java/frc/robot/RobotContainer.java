// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood.HoodConstants;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.Indexer.KickerSubsystem;
import frc.robot.subsystems.Indexer.SpindexerSubsystem;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakePivotSubsystem;
import frc.robot.subsystems.Intake.IntakeWheelsSubsystem;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;

@Logged
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            // .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final CommandXboxController debugController = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final TurretSubsystem turret = new TurretSubsystem();
    public final HoodSubsystem hood = new HoodSubsystem();
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    // public final ClimberSubsystem climber = new ClimberSubsystem();
    public final IntakePivotSubsystem pivot = new IntakePivotSubsystem();
    public final IntakeWheelsSubsystem wheels = new IntakeWheelsSubsystem();
    public final SpindexerSubsystem spindexer = new SpindexerSubsystem();
    public final KickerSubsystem kicker = new KickerSubsystem();

    Trigger withinShooterTolerance = new Trigger(turret::turretAtPosition);
    Trigger withinPassingTolerance = new Trigger(turret::turretAtPassing);

    public final AutoAim autoAimCommandFactory = new AutoAim(drivetrain, turret, hood, shooter, spindexer, kicker);

    public SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        SmartDashboard.putNumber("set hood", 18.0);
        SmartDashboard.putNumber("set shooter", 0);
        SmartDashboard.putNumber("set turret", 0);
        registerCommands();
        configureDriverControls();
        configureOperatorControls();
        configureDebugControls();
        configureDefaultCommands();
    }

    private void registerCommands() {
        NamedCommands.registerCommand("intake",
            Commands.run(() -> wheels.setWheelVoltage(10), wheels)
            .alongWith(Commands.run(() -> pivot.usePivotPID(IntakeConstants.pivotDeployPosition), pivot)));

        NamedCommands.registerCommand("shoot",
            autoAimCommandFactory.generateTurretIdleCommand()
            .alongWith(autoAimCommandFactory.generateHoodIdleCommand())
            .alongWith(autoAimCommandFactory.generateAssumedShooterCommand())
            .alongWith(Commands.waitSeconds(ShooterConstants.shooterRampTimeSeconds).andThen(Commands.run(() -> spindexer.setSpindexerVolts(8), spindexer)))
            .alongWith(Commands.waitSeconds(ShooterConstants.shooterRampTimeSeconds).andThen(Commands.run(() -> kicker.setKickerVolts(12), kicker)))
            .withTimeout(10.0)
            .finallyDo(() -> {spindexer.setSpindexerVolts(0); kicker.setKickerVolts(0); shooter.setShooterVoltage(3);}));

        NamedCommands.registerCommand("shootFast",
            autoAimCommandFactory.generateTurretIdleCommand()
            .alongWith(autoAimCommandFactory.generateHoodIdleCommand())
            .alongWith(autoAimCommandFactory.generateAssumedShooterCommand())
            .alongWith(Commands.run(() -> spindexer.setSpindexerVolts(8), spindexer))
            .alongWith(Commands.run(() -> kicker.setKickerVolts(12), kicker))
            .alongWith(Commands.waitSeconds(3.0).andThen(Commands.run(() -> pivot.usePivotPID(12.0 / 82.74 * 360.0), pivot)))
            .withTimeout(4.0)
            .finallyDo(() -> {spindexer.setSpindexerVolts(0); kicker.setKickerVolts(0);}));
            
        NamedCommands.registerCommand("shootIdle", Commands.runOnce(() -> shooter.setShooterVoltage(8), shooter));

        NamedCommands.registerCommand("turretFlip", turret.getTurretPIDCommand(() -> 70.0));
        NamedCommands.registerCommand("turretZero", turret.getTurretPIDCommand(() -> -100.0));
        NamedCommands.registerCommand("stow", hood.getHoodPIDCommand(() -> 18));
        autoChooser.setDefaultOption("none", Commands.waitSeconds(0));
        autoChooser.addOption("Right smash (OG)", AutoBuilder.buildAuto("OutpostTwo"));
        autoChooser.addOption("Left smash (OG)", AutoBuilder.buildAuto("DepotTwo"));
        autoChooser.addOption("Right simple (no passback)", AutoBuilder.buildAuto("OutpostSimple"));
        autoChooser.addOption("Left Simple (no passback)", AutoBuilder.buildAuto("DepotSimple"));
        autoChooser.addOption("Right mix (og + simple)", AutoBuilder.buildAuto("OutpostMix"));
        autoChooser.addOption("Left Mix (og + simple)", AutoBuilder.buildAuto("DepotMix"));
        autoChooser.addOption("Right Bump", AutoBuilder.buildAuto("OutpostBump"));
        // autoChooser.addOption("Left Bump", AutoBuilder.buildAuto("OutpostBump"));
        autoChooser.addOption("sert", AutoBuilder.buildAuto("sert"));
        autoChooser.addOption("OrbitOutpost", AutoBuilder.buildAuto("OrbitOutpost"));
        // autoChooser.addOption("OutpostBump", AutoBuilder.buildAuto("OutpostBump"));
        // autoChooser.addOption("DepotBump", AutoBuilder.buildAuto("DepotBump"));
        autoChooser.addOption("test", AutoBuilder.buildAuto("test"));
        SmartDashboard.putData(autoChooser);
    }

    private void configureDriverControls() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Reset the field-centric heading
        driverController.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        driverController.rightTrigger().whileTrue(autoAimCommandFactory.generateAssumedShooterCommand())
            .whileFalse(Commands.run(() -> shooter.setShooterVoltage(0), shooter));
        driverController.rightTrigger().whileTrue(autoAimCommandFactory.generateHoodIdleCommand())
            .whileFalse(hood.getHoodPIDCommand(() -> 18.0));
        driverController.rightTrigger().and(withinShooterTolerance).whileTrue(Commands.waitSeconds(ShooterConstants.shooterRampTimeSeconds).andThen(Commands.run(() -> spindexer.setSpindexerVolts(8), spindexer)));
        driverController.rightTrigger().and(withinShooterTolerance).whileTrue(Commands.waitSeconds(ShooterConstants.shooterRampTimeSeconds).andThen(Commands.run(() -> kicker.setKickerVolts(12), kicker)));
        driverController.rightTrigger().onTrue(autoAimCommandFactory.generateTurretIdleCommand());
        driverController.rightTrigger().onFalse(shooter.getShooterPIDCommand(() -> ShooterConstants.shooterIdleRPM));

        driverController.rightBumper().whileTrue(autoAimCommandFactory.generateSOTMScoringCommand());
        driverController.rightBumper().and(withinShooterTolerance).whileTrue(Commands.waitSeconds(ShooterConstants.shooterRampTimeSeconds).andThen(Commands.run(() -> spindexer.setSpindexerVolts(8), spindexer)));
        driverController.rightBumper().and(withinShooterTolerance).whileTrue(Commands.waitSeconds(ShooterConstants.shooterRampTimeSeconds).andThen(Commands.run(() -> kicker.setKickerVolts(12), kicker)));
        driverController.rightBumper().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed / 2) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed / 2) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate / 2) // Drive counterclockwise with negative X (left)
        ));
        driverController.rightBumper().onFalse(autoAimCommandFactory.generateTurretIdleCommand());
        driverController.rightBumper().onFalse(shooter.getShooterPIDCommand(() -> ShooterConstants.shooterIdleRPM));
        driverController.leftBumper().onTrue(Commands.run(() -> wheels.setWheelVoltage(-10), wheels));
        driverController.leftBumper().onFalse(Commands.run(() -> wheels.setWheelVoltage(0), wheels));
        driverController.leftBumper().onTrue(Commands.run(() -> pivot.usePivotPID(IntakeConstants.pivotDeployPosition), pivot));


        driverController.leftTrigger().onTrue(Commands.run(() -> wheels.setWheelVoltage(10), wheels));
        driverController.leftTrigger().onFalse(Commands.run(() -> wheels.setWheelVoltage(0), wheels));
        driverController.leftTrigger().onTrue(Commands.run(() -> pivot.usePivotPID(IntakeConstants.pivotDeployPosition), pivot));
        // driverController.leftTrigger().onTrue(autoAimCommandFactory.generateTurretIdleCommand());
        driverController.povLeft().whileTrue(drivetrain.applyRequest(() -> drivetrain.commandChassisSpeeds(new ChassisSpeeds(1.0, 0, 0))));
    }

    private void configureOperatorControls() {
        // operatorController.a().whileTrue(hood.getHoodPIDCommand(() -> HoodConstants.hoodStowSetpoint));

        // operatorController.povUp().onTrue(Commands.run(() -> climber.useClimberPID(ClimberConstants.deployedPosition), climber));
        // operatorController.povDown().onTrue(Commands.run(() -> climber.useClimberPID(ClimberConstants.climbedPosition), climber));
        
        operatorController.rightTrigger().whileTrue(Commands.run(() -> pivot.usePivotPID(IntakeConstants.pivotStowPosiion), pivot));
        operatorController.rightTrigger().onTrue(Commands.run(() -> turret.manualTurret(0), turret));
        operatorController.rightTrigger().onTrue(Commands.run(() -> shooter.setShooterVoltage(0), shooter));
        
        operatorController.leftTrigger().whileTrue(Commands.run(() -> pivot.usePivotPID(12.0 / 82.74 * 360.0), pivot));
        operatorController.leftBumper().whileTrue(Commands.run(() -> pivot.usePivotPID(IntakeConstants.pivotDeployPosition), pivot));


        operatorController.a().whileTrue(turret.getTurretPIDCommand(() -> -90));
        // operatorController.a().toggleOnTrue(autoAimCommandFactory.generateHoodIdleCommand());
        operatorController.y().whileTrue(hood.getHoodPIDCommand(() -> HoodConstants.hoodStowSetpoint));
    }

    private void configureDebugControls() {
        debugController.a().whileTrue(Commands.defer(() -> hood.getHoodPIDCommand(() -> SmartDashboard.getNumber("set hood", 18.0)), Set.of(hood)));
        debugController.a().whileTrue(Commands.defer(() -> shooter.getShooterPIDCommand(() -> SmartDashboard.getNumber("set shooter", 0)), Set.of(shooter)));
        debugController.b().whileTrue(autoAimCommandFactory.generateTurretIdleCommand());
        debugController.y().whileTrue(Commands.run(() -> pivot.usePivotPID(IntakeConstants.pivotStowPosiion), pivot));

        debugController.povUp().whileTrue(turret.sysIdDynamic(Direction.kForward));
        debugController.povRight().whileTrue(turret.sysIdDynamic(Direction.kReverse));
        debugController.povDown().whileTrue(turret.sysIdQuasistatic(Direction.kForward));
        debugController.povLeft().whileTrue(turret.sysIdQuasistatic(Direction.kReverse));


        debugController.rightTrigger().whileTrue(Commands.run(() -> spindexer.setSpindexerVolts(8), spindexer));
        debugController.rightTrigger().whileTrue(Commands.run(() -> kicker.setKickerVolts(12), kicker));
    }

    public void configureDefaultCommands() {
        // shooter.setDefaultCommand(shooter.getShooterPIDCommand(() -> ShooterConstants.shooterIdleRPM));
        // climber.setDefaultCommand(Commands.run(() -> climber.setClimberVoltage(0), climber));
        pivot.setDefaultCommand(Commands.run(() -> pivot.setPivotVoltage(0), pivot));
        wheels.setDefaultCommand(Commands.run(() -> wheels.setWheelVoltage(0), wheels));
        spindexer.setDefaultCommand(Commands.run(() -> spindexer.setSpindexerVolts(0), spindexer));
        kicker.setDefaultCommand(Commands.run(() -> kicker.setKickerVolts(0), kicker));
        
        turret.setDefaultCommand(Commands.run(() -> turret.manualTurret(0), turret));
        // turret.setDefaultCommand(autoAimCommandFactory.generateTurretIdleCommand());
        hood.setDefaultCommand(hood.getHoodPIDCommand(() -> 18));
        shooter.setDefaultCommand(Commands.run(() -> shooter.setShooterVoltage(0), shooter));
    }

    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
