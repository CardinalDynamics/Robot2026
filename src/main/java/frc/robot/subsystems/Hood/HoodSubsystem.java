package frc.robot.subsystems.Hood;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility.ShooterParameters;

@Logged
public class HoodSubsystem extends SubsystemBase {

    TalonFX hoodMotor;
    TalonFXConfiguration motorConfig;
    MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    double desiredHoodPosition;

    private final DCMotorSim m_motorSimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX44Foc(1), 0.001, HoodConstants.gearRatio
    ),
    DCMotor.getKrakenX44Foc(1)
    );
    
    public HoodSubsystem() {
        hoodMotor = new TalonFX(HoodConstants.hoodMotorCANID, Constants.canivoreBus);

        // Config settings for the x44
        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        Slot0Configs slot0 = motorConfig.Slot0;
        slot0.kP = HoodConstants.kP;
        slot0.kI = HoodConstants.kI;
        slot0.kD = HoodConstants.kD;
        slot0.kS = HoodConstants.kS;
        slot0.kV = HoodConstants.kV;
        slot0.kA = HoodConstants.kA;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.MotionMagicCruiseVelocity;
        motorConfig.MotionMagic.MotionMagicAcceleration = HoodConstants.MotionMagicAcceleration;

        // apply config
        hoodMotor.getConfigurator().apply(motorConfig);

        // set some sim settings
        var hoodMotorSim = hoodMotor.getSimState();
        hoodMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
        hoodMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);

        // set offset for hood
        hoodMotor.setPosition(0);
        desiredHoodPosition = getHoodDegrees();
    }
    
    // get the position of the hood in degrees
    public double getHoodDegrees() {
        return hoodMotor.getPosition().getValueAsDouble() * 360.0 / HoodConstants.gearRatio + HoodConstants.hoodOffset;
    }

    public double getDesiredHoodDegrees() {
        return desiredHoodPosition * 360.0 / HoodConstants.gearRatio + HoodConstants.hoodOffset;
    }

    private void setDesiredHoodAngle(double rotations) {
        desiredHoodPosition = rotations;
    }

    // use motion magic to move the hood to desired angle
    public Command getHoodPIDCommand(DoubleSupplier degrees) {
        return run(() -> hoodMotor.setControl(motionMagicRequest.withPosition(
            MathUtil.clamp(degrees.getAsDouble(), HoodConstants.hoodMinLimit, HoodConstants.hoodMaxLimit) * HoodConstants.gearRatio / 360.0)))
            .alongWith(Commands.run(() -> setDesiredHoodAngle(MathUtil.clamp(degrees.getAsDouble(), HoodConstants.hoodMinLimit, HoodConstants.hoodMaxLimit) * HoodConstants.gearRatio / 360.0)));
    }

    public Command getHoodPIDCommand(Supplier<ShooterParameters> params) {
        return run(() -> 
            hoodMotor.setControl(motionMagicRequest.withPosition(
                MathUtil.clamp(params.get().getHoodAngle(), HoodConstants.hoodMinLimit, HoodConstants.hoodMaxLimit) * HoodConstants.gearRatio / 360.0)))
                .alongWith(Commands.run(() -> setDesiredHoodAngle(MathUtil.clamp(params.get().getHoodAngle(), HoodConstants.hoodMinLimit, HoodConstants.hoodMaxLimit) * HoodConstants.gearRatio / 360.0)));
    }

    public boolean getHoodAtPosition() {
        return Math.abs(getHoodDegrees() - getDesiredHoodDegrees()) < HoodConstants.hoodTolerance;
    }

    public void manualHood(double volts) {
        hoodMotor.setVoltage(volts);
    }

    public double getShotAngle() {
        return 180.0 - (getHoodDegrees() + 90.0);
    }
    

    @Override
    public void simulationPeriodic() {
        var talonFXSim = hoodMotor.getSimState();

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
        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(HoodConstants.gearRatio));
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(HoodConstants.gearRatio));

        SmartDashboard.putNumber(
            "Hood Motor Volts",
            hoodMotor.getSimState().getMotorVoltageMeasure().in(Volts));
    }

}
