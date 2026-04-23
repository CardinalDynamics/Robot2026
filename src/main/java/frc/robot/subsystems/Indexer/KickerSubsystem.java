package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {

    TalonFX kickerMotor;
    TalonFXConfiguration motorConfig;
    VoltageOut voltageOutReq = new VoltageOut(0).withEnableFOC(true);
    
    public KickerSubsystem() {
        kickerMotor = new TalonFX(IndexerConstants.kickerMotorCANID, Constants.rioBus);

        // Config settings for the x60
        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;

        // apply config
        kickerMotor.getConfigurator().apply(motorConfig);
    }

    // public void setWheelCurrent(double amps) {
    //     intakeWheels.setControl(torqueRequest.withOutput(amps));
    // }

    public void setKickerVolts(double volts) {
        kickerMotor.setVoltage(volts);
    }

    public Object setSpindexerVoltage(int i) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSpindexerVoltage'");
    }
}
