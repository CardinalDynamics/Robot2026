package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class TurretSubsystem extends SubsystemBase {

    TalonFX turretMotor;
    VelocityVoltage voltageRequest;
    TalonFXConfiguration motorConfig;
    MotionMagicTorqueCurrentFOC motionMagicRequest = new MotionMagicTorqueCurrentFOC(0);
    
    public TurretSubsystem() {
        turretMotor = new TalonFX(TurretConstants.turretMotorCANID, Constants.canivoreBus);
        voltageRequest = new VelocityVoltage(0);

        // Config settings for the x44
        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        Slot0Configs slot0 = motorConfig.Slot0;
        slot0.kP = TurretConstants.kP;
        slot0.kI = TurretConstants.kI;
        slot0.kD = TurretConstants.kD;
        slot0.kS = TurretConstants.kS;
        slot0.kV = TurretConstants.kV;
        slot0.kA = TurretConstants.kA;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MotionMagicCruiseVelocity;
        motorConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.MotionMagicAcceleration;

        // apply config
        turretMotor.getConfigurator().apply(motorConfig);

        // set offset for turret
        turretMotor.setPosition(TurretConstants.turretOffset);
    }

    // get the position of the turret in degrees
    public double getTurretDegrees() {
        return turretMotor.getPosition().getValueAsDouble() * 360.0 / TurretConstants.gearRatio;
    }

    public void manualTurret(double input) {
        turretMotor.setControl(voltageRequest.withVelocity(input));
    }

    // use motion magic to move the turret to desired angle
    public Command getTurretPIDCommand(double degrees) {
        return run(() -> turretMotor.setControl(motionMagicRequest.withPosition(degrees * TurretConstants.gearRatio / 360.0)));
    }

    // convert angles in (0, 360) to (-180, 180)
    public double normalizeDegrees(double degrees) {
        return (degrees + 180.0) % 360.0 - 180.0;
    }

    // select the target angle closest to the current angle and within the range of the turret
    public double degreesToTurretAngle(double target) {
        double error = normalizeDegrees(normalizeDegrees(target) - getTurretDegrees());
        double wrappedTarget = getTurretDegrees() + error;
        if (wrappedTarget > TurretConstants.clockwiseTurretLimitDegrees) {
            wrappedTarget -= 360.0;
        } else if (wrappedTarget < TurretConstants.counterclockwiseTurretLimitDegrees) {
            wrappedTarget += 360.0;
        }
        return wrappedTarget;
    }

    // get the desired shot angle based on the position of the robot
    // then translates to the turret angle needed to achieve the shot angle based on the rotation of the robot
    public double getDesiredTurretAngle(Pose2d targetPose, Pose2d drivePose) {
        Rotation2d desiredShotAngle = Rotation2d.fromRadians(Math.atan2(targetPose.getY() - drivePose.getY(),
            targetPose.getX() - drivePose.getX()));
        double turretAngleDegrees = desiredShotAngle.getDegrees() - drivePose.getRotation().getDegrees();
        return turretAngleDegrees;
    }
}