package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeWheelsSubsystem extends SubsystemBase {

    TalonFX intakeWheels;
    TalonFXConfiguration motorConfig;
    VoltageOut voltageOutReq = new VoltageOut(0).withEnableFOC(false);
    
    public IntakeWheelsSubsystem() {
        intakeWheels = new TalonFX(IntakeConstants.wheelMotorCANID, Constants.canivoreBus);

        // Config settings for the x60
        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;

        // apply config
        intakeWheels.getConfigurator().apply(motorConfig);
    }

    // public void setWheelCurrent(double amps) {
    //     intakeWheels.setControl(torqueRequest.withOutput(amps));
    // }

    public void setWheelVoltage(double volts) {
        intakeWheels.setVoltage(volts);
    }
}