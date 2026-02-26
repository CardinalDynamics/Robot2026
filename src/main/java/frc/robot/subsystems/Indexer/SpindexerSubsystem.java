package frc.robot.subsystems.Indexer;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class SpindexerSubsystem extends SubsystemBase {
    SparkMax SpindexerMotor;
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    ProfiledPIDController controller;

    public SpindexerSubsystem() {
        SpindexerMotor = new SparkMax(IndexerConstants.spindexerMotorCANID, MotorType.kBrushless);
        motorConfig.idleMode(IdleMode.kBrake);
        SpindexerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        controller = new ProfiledPIDController(IndexerConstants.kSpindexerP, IndexerConstants.kSpindexerI, IndexerConstants.kSpindexerD,
            new Constraints(IndexerConstants.SpindexerMaxVelocity, IndexerConstants.SpindexerMaxAcceleration));
    }

    public double getSpindexerRPM() {
        return SpindexerMotor.getEncoder().getVelocity();
    }

    public double getControllerSetpoint() {
        return controller.getSetpoint().position;
    }

    public void useSpindexerPID(double rpm) {
        controller.setGoal(rpm);
        SpindexerMotor.setVoltage(controller.calculate(getSpindexerRPM()));
    }

    public void setSpindexerVoltage(double volts) {
        SpindexerMotor.setVoltage(volts);
    }

    public Command getSpindexerCommand(DoubleSupplier rpm) {
        return run(() -> SpindexerMotor.setVoltage(controller.calculate(getSpindexerRPM())))
        .alongWith(Commands.run(() -> controller.setGoal(rpm.getAsDouble())));
    }
}