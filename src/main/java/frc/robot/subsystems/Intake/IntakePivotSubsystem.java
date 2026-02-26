package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import frc.robot.subsystems.Hood.HoodConstants;

@Logged
public class IntakePivotSubsystem extends SubsystemBase {

    TalonFX pivotMotor;
    TalonFXConfiguration motorConfig;
    MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private final DCMotorSim m_motorSimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60Foc(1), 0.001, IntakeConstants.pivotGearRatio
    ),
    DCMotor.getKrakenX60Foc(1)
    );
    
    public IntakePivotSubsystem() {
        pivotMotor = new TalonFX(IntakeConstants.pivotMotorCANDID, Constants.canivoreBus);

        // Config settings for the x44
        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        Slot0Configs slot0 = motorConfig.Slot0;
        slot0.kP = IntakeConstants.kPivotP;
        slot0.kI = IntakeConstants.kPivotI;
        slot0.kD = IntakeConstants.kPivotD;
        slot0.kS = IntakeConstants.kPivotS;
        slot0.kV = IntakeConstants.kPivotV;
        slot0.kA = IntakeConstants.kPivotA;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.PivotMotionMagicCruiseVelocity;
        motorConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.PivotMotionMagicAcceleration;

        // apply config
        pivotMotor.getConfigurator().apply(motorConfig);

        // set some sim settings
        var pivotMotorSim = pivotMotor.getSimState();
        pivotMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
        pivotMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);

        // set offset for pivot
        pivotMotor.setPosition(IntakeConstants.pivotOffset);
    }

    // get the position of the pivot in degrees
    public double getPivotDegrees() {
        return pivotMotor.getPosition().getValueAsDouble() * 360.0 / IntakeConstants.pivotGearRatio;
    }

    public void setPivotVelocity(double velocity) {
        pivotMotor.setControl(velocityRequest.withVelocity(velocity));
    }

    public void usePivotPID(double degrees) {
        pivotMotor.setControl(motionMagicRequest.withPosition(degrees * HoodConstants.gearRatio / 360.0));
    }

    public void setPivotVoltage(double volts) {
        pivotMotor.setVoltage(volts);
    }
    
    @Override
    public void simulationPeriodic() {
        var talonFXSim = pivotMotor.getSimState();

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
        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(IntakeConstants.pivotGearRatio));
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(IntakeConstants.pivotGearRatio));

        SmartDashboard.putNumber(
            "Pivot Motor Volts",
            pivotMotor.getSimState().getMotorVoltageMeasure().in(Volts));
    }
}
