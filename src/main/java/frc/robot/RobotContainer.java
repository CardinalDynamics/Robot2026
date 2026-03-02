// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
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
    public final ClimberSubsystem climber = new ClimberSubsystem();
    public final IntakePivotSubsystem pivot = new IntakePivotSubsystem();
    public final IntakeWheelsSubsystem wheels = new IntakeWheelsSubsystem();
    public final SpindexerSubsystem spindexer = new SpindexerSubsystem();
    public final KickerSubsystem kicker = new KickerSubsystem();

    public final AutoAim autoAimCommandFactory = new AutoAim(drivetrain, turret, hood, shooter, spindexer, kicker);

    public RobotContainer() {
        SmartDashboard.putNumber("set hood", 18.0);
        SmartDashboard.putNumber("set shooter", 0);
        SmartDashboard.putNumber("set turret", 0);
        configureDriverControls();
        configureOperatorControls();
        configureDebugControls();
        configureDefaultCommands();
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

        // Reset the field-centric heading on left bumper press.
        driverController.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        driverController.rightTrigger().whileTrue(autoAimCommandFactory.generateAssumedShooterCommand());
        driverController.rightTrigger().whileTrue(Commands.run(() -> spindexer.setSpindexerVoltage(8), spindexer));
        driverController.rightTrigger().whileTrue(Commands.run(() -> kicker.setSpindexerVoltage(12), kicker));

        driverController.leftTrigger().whileTrue(Commands.run(() -> wheels.setIntakeWheelVoltage(-4), wheels));
        driverController.leftTrigger().whileTrue(Commands.run(() -> pivot.usePivotPID(IntakeConstants.pivotDeployPosition), pivot));
    }

    private void configureOperatorControls() {
        // operatorController.a().whileTrue(hood.getHoodPIDCommand(() -> HoodConstants.hoodStowSetpoint));

        operatorController.povUp().onTrue(Commands.run(() -> climber.useClimberPID(ClimberConstants.deployedPosition), climber));
        operatorController.povDown().onTrue(Commands.run(() -> climber.useClimberPID(ClimberConstants.climbedPosition), climber));
        operatorController.rightTrigger().whileTrue(Commands.run(() -> pivot.usePivotPID(IntakeConstants.pivotStowPosiion), pivot));
        operatorController.leftTrigger().whileTrue(Commands.run(() -> pivot.usePivotPID(IntakeConstants.pivotDeployPosition), pivot));
        operatorController.rightBumper().whileTrue(Commands.run(() -> wheels.useIntakeWheelPID(IntakeConstants.IntakeSpeed), wheels));
        operatorController.leftBumper().whileTrue(Commands.run(() -> wheels.useIntakeWheelPID(-IntakeConstants.IntakeSpeed * .5), wheels));

        operatorController.a().toggleOnTrue(autoAimCommandFactory.generateTurretIdleCommand());
        operatorController.a().toggleOnTrue(autoAimCommandFactory.generateHoodIdleCommand());
        operatorController.a().toggleOnTrue(shooter.getShooterPIDCommand(() -> ShooterConstants.shooterIdleRPM));
        
    }

    private void configureDebugControls() {
        debugController.a().whileTrue(Commands.defer(() -> hood.getHoodPIDCommand(() -> SmartDashboard.getNumber("set hood", 18.0)), Set.of(hood)));
        debugController.a().whileTrue(Commands.defer(() -> shooter.getShooterPIDCommand(() -> SmartDashboard.getNumber("set shooter", 0)), Set.of(shooter)));
        debugController.a().whileTrue(Commands.defer(() -> turret.getTurretPIDCommand(() -> turret.degreesToTurretAngle(SmartDashboard.getNumber("set turret", 0))), Set.of(turret)));
        debugController.y().whileTrue(Commands.run(() -> pivot.usePivotPID(IntakeConstants.pivotStowPosiion), pivot));

        debugController.povUp().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        debugController.povRight().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        debugController.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        debugController.povLeft().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    public void configureDefaultCommands() {
        // shooter.setDefaultCommand(shooter.getShooterPIDCommand(() -> ShooterConstants.shooterIdleRPM));
        climber.setDefaultCommand(Commands.run(() -> climber.setClimberVoltage(0), climber));
        pivot.setDefaultCommand(Commands.run(() -> pivot.setPivotVoltage(0), pivot));
        wheels.setDefaultCommand(Commands.run(() -> wheels.setIntakeWheelVoltage(0), wheels));
        spindexer.setDefaultCommand(Commands.run(() -> spindexer.setSpindexerVoltage(0), spindexer));
        kicker.setDefaultCommand(Commands.run(() -> kicker.setSpindexerVoltage(0), kicker));
        
        // turret.setDefaultCommand(Commands.run(() -> turret.manualTurret(0), turret));
        turret.setDefaultCommand(autoAimCommandFactory.generateTurretIdleCommand());
        hood.setDefaultCommand(Commands.run(() -> hood.manualHood(0), hood));
        shooter.setDefaultCommand(shooter.getShooterPIDCommand(() -> ShooterConstants.shooterIdleRPM));
    }

    
    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
