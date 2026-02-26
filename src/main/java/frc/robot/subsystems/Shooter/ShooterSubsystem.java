package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
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
public class ShooterSubsystem extends SubsystemBase {

    TalonFX shooterMotorLeft;
    TalonFX shooterMotorRight;
    TalonFXConfiguration motorConfigLeft;
    TalonFXConfiguration motorConfigRight;
    MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(0);
    double desiredShooterSpeed = 0;

    private final DCMotorSim m_motorSimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60Foc(1), 0.001, 1
    ),
    DCMotor.getKrakenX60Foc(1)
    );
    
    public ShooterSubsystem() {
        shooterMotorRight = new TalonFX(ShooterConstants.shooterMotorRightCANID, Constants.canivoreBus);
        shooterMotorLeft = new TalonFX(ShooterConstants.shooterMotorLeftCANID, Constants.canivoreBus);

        // Config settings for the x60
        motorConfigRight = new TalonFXConfiguration();
        motorConfigLeft = new TalonFXConfiguration();
        motorConfigRight.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        Slot0Configs slot0 = motorConfigRight.Slot0;
        slot0.kP = ShooterConstants.kP;
        slot0.kI = ShooterConstants.kI;
        slot0.kD = ShooterConstants.kD;
        slot0.kS = ShooterConstants.kS;
        slot0.kV = ShooterConstants.kV;
        slot0.kA = ShooterConstants.kA;
        motorConfigRight.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.MotionMagicCruiseVelocity;
        motorConfigRight.MotionMagic.MotionMagicAcceleration = ShooterConstants.MotionMagicAcceleration;

        motorConfigRight.MotorOutput.Inverted = ShooterConstants.shooterMotorRightInvertedValue;
        motorConfigLeft.MotorOutput.Inverted = ShooterConstants.shooterMotorLeftInvertedValue;

        // apply config
        shooterMotorLeft.getConfigurator().apply(motorConfigLeft);
        shooterMotorRight.getConfigurator().apply(motorConfigRight);

        // set some sim settings
        var shooterMotorSim = shooterMotorLeft.getSimState();
        shooterMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
        shooterMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        shooterMotorLeft.setControl(new Follower(ShooterConstants.shooterMotorRightCANID, MotorAlignmentValue.Opposed));
    }

    public double getDesiredShooterRPM() {
        return desiredShooterSpeed * 60.0;
    }

    private void setDesiredShooterSpeed(double rps) {
        desiredShooterSpeed = rps;
    }

    public boolean getShooterAtSpeed() {
        return Math.abs(getDesiredShooterRPM() - getShooterSpeed()) < ShooterConstants.shooterToleranceRPM;
    }
    
    // get the rotational velocity of the shooter wheels in RPM
    public double getShooterSpeed() {
        return shooterMotorRight.getVelocity().getValueAsDouble() * 60.0;
    }

    // use motion magic to change the speed of the shooter wheels
    public Command getShooterPIDCommand(DoubleSupplier rpm) {
        return run(() -> {
            shooterMotorRight.setControl(motionMagicRequest.withVelocity(rpm.getAsDouble() / 60.0));
        }).alongWith(Commands.run(() -> setDesiredShooterSpeed(rpm.getAsDouble() / 60.0)));
    }

    public Command getShooterPIDCommand(Supplier<ShooterParameters> params) {
        return run(() -> {
            shooterMotorRight.setControl(motionMagicRequest.withVelocity(params.get().shooterSpeedRPM / 60.0));
        }).alongWith(Commands.run(() -> setDesiredShooterSpeed(params.get().shooterSpeedRPM / 60.0)));
    }

    public void setShooterVoltage(double volts) {
        shooterMotorRight.setVoltage(volts);
    }


    @Override
    public void simulationPeriodic() {
        var talonFXSim = shooterMotorLeft.getSimState();

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
        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition());
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity());

        SmartDashboard.putNumber(
            "Hood Motor Volts",
            shooterMotorLeft.getSimState().getMotorVoltageMeasure().in(Volts));
    }

}