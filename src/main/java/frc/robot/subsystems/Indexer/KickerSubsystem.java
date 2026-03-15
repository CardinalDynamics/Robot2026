package frc.robot.subsystems.Indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
    SparkMax kickerMotor;
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    ProfiledPIDController controller;

    public KickerSubsystem() {
        kickerMotor = new SparkMax(IndexerConstants.kickerMotorCANID, MotorType.kBrushless);
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.inverted(true);
        motorConfig.smartCurrentLimit(40);
        kickerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        controller = new ProfiledPIDController(IndexerConstants.kKickerP, IndexerConstants.kKickerI, IndexerConstants.kKickerD,
            new Constraints(IndexerConstants.KickerMaxVelocity, IndexerConstants.KickerMaxAcceleration));
    }

    public double getKickerRPM() {
        return kickerMotor.getEncoder().getVelocity() / IndexerConstants.kickerGearRatio;
    }

    public double getControllerSetpoint() {
        return controller.getSetpoint().position;
    }

    public void useKickerPID(double rpm) {
        controller.setGoal(rpm);
        kickerMotor.setVoltage(controller.calculate(getKickerRPM()));
    }

    public void setSpindexerVoltage(double volts) {
        kickerMotor.setVoltage(volts);
    }
}
