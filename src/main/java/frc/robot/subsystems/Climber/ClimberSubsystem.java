package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class ClimberSubsystem extends SubsystemBase {
    TalonFX climbMotor;
    TalonFXConfiguration motorConfig;
    MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private final DCMotorSim m_motorSimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60Foc(1), 0.001, ClimberConstants.gearRatio
    ),
    DCMotor.getKrakenX60Foc(1)
    );
    
    public ClimberSubsystem() {
        climbMotor = new TalonFX(ClimberConstants.climbMotorCANID, Constants.canivoreBus);

        // Config settings for the x44
        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        Slot0Configs slot0 = motorConfig.Slot0;
        slot0.kP = ClimberConstants.kP;
        slot0.kI = ClimberConstants.kI;
        slot0.kD = ClimberConstants.kD;
        slot0.kS = ClimberConstants.kS;
        slot0.kV = ClimberConstants.kV;
        slot0.kA = ClimberConstants.kA;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.MotionMagicCruiseVelocity;
        motorConfig.MotionMagic.MotionMagicAcceleration = ClimberConstants.MotionMagicAcceleration;

        // apply config
        climbMotor.getConfigurator().apply(motorConfig);

        // set some sim settings
        var climbMotorSim = climbMotor.getSimState();
        climbMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
        climbMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        climbMotor.setPosition(60);
    }

    public double getClimberPosition() {
        return climbMotor.getPosition().getValueAsDouble();
    }

    public void useClimberPID(double position) {
        climbMotor.setControl(motionMagicRequest.withPosition(position));
    }

    public void setClimberVoltage(double volts) {
        climbMotor.setVoltage(volts);
    }

    @Override
    public void simulationPeriodic() {
        var talonFXSim = climbMotor.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(ClimberConstants.gearRatio));
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(ClimberConstants.gearRatio));

        SmartDashboard.putNumber(
            "Climb Motor Volts",
            climbMotor.getSimState().getMotorVoltageMeasure().in(Volts));
    }

}
