package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWheelsSubsystem extends SubsystemBase {
    SparkMax wheelMotor;
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    ProfiledPIDController controller;

    public IntakeWheelsSubsystem() {
        wheelMotor = new SparkMax(IntakeConstants.wheelMotorCANID, MotorType.kBrushless);
        motorConfig.idleMode(IdleMode.kCoast);
        wheelMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        controller = new ProfiledPIDController(IntakeConstants.kWheelP, IntakeConstants.kWheelI, IntakeConstants.kWheelD,
            new Constraints(IntakeConstants.IntakeWheelMaxVelocity, IntakeConstants.IntakeWheelMaxAcceleration));
    }

    public double getIntakeWheelRPM() {
        return wheelMotor.getEncoder().getVelocity() / IntakeConstants.wheelGearRatio;
    }

    public double getControllerSetpoint() {
        return controller.getSetpoint().position;
    }

    public void useIntakeWheelPID(double rpm) {
        controller.setGoal(rpm);
        wheelMotor.setVoltage(controller.calculate(getIntakeWheelRPM()));
    }

    public void setIntakeWheelVoltage(double volts) {
        wheelMotor.setVoltage(volts);
    }
}