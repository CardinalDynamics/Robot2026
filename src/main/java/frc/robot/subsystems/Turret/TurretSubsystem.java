package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    TalonFX turretMotor;
    VelocityVoltage voltageRequest = new VelocityVoltage(0);
    TalonFXConfiguration motorConfig;
    PositionVoltage positionRequest = new PositionVoltage(0);

    private final DCMotorSim m_motorSimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX44Foc(1), 0.001, TurretConstants.gearRatio
    ),
    DCMotor.getKrakenX44Foc(1)
    );

    double desiredPosition = 90.0;

    private final SysIdRoutine turretRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(.8).per(Seconds),        // Use default ramp rate (1 V/s)
            Volts.of(2.5), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysID_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> manualTurret(output.in(Volts)),
            null,
            this
        )
    );
    
    public TurretSubsystem() {
        turretMotor = new TalonFX(TurretConstants.turretMotorCANID, Constants.canivoreBus);

        // Config settings for the x44
        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.ClosedLoopGeneral.GainSchedErrorThreshold = 0;
        Slot0Configs slot0 = motorConfig.Slot0;
        Slot1Configs slot1 = motorConfig.Slot1;
        slot0.kP = TurretConstants.kP;
        slot0.kI = TurretConstants.kI;
        slot0.kD = TurretConstants.kD;
        slot0.kS = TurretConstants.kS;
        slot0.kV = TurretConstants.kV;
        slot0.kA = TurretConstants.kA;
        slot0.GainSchedBehavior = GainSchedBehaviorValue.UseSlot1;
        slot1.kP = 7;
        slot1.kI = TurretConstants.kI;
        slot1.kD = 0;
        slot1.kS = TurretConstants.kS;
        slot1.kV = TurretConstants.kV;
        slot1.kA = TurretConstants.kA;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MotionMagicCruiseVelocity;
        motorConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.MotionMagicAcceleration;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        // apply config
        turretMotor.getConfigurator().apply(motorConfig.withSlot1(slot1));

        // set some sim settings
        var turretMotorSim = turretMotor.getSimState();
        turretMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
        turretMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);

        // set offset for turret
        turretMotor.setPosition(TurretConstants.gearRatio * TurretConstants.turretOffset / 360.0);
    }

    public void setDesiredPosition(double degrees) {
        desiredPosition = degrees;
    }

    // get the position of the turret in degrees
    @Logged
    public double getTurretDegrees() {
        return turretMotor.getPosition().getValueAsDouble() * 360.0 / TurretConstants.gearRatio;
    }

    public boolean turretAtPosition() {
        return Math.abs(angleError(getTurretDegrees(), desiredPosition)) < 3.0;
    }

    public boolean turretAtPassing() {
        return Math.abs(angleError(getTurretDegrees(), desiredPosition)) < 3.0;
    }

    public void manualTurret(double input) {
        turretMotor.setVoltage(input);
    }

    // use motion magic to move the turret to desired angle
    public Command getTurretPIDCommand(DoubleSupplier degrees) {
        return run(() -> turretMotor.setControl(positionRequest.withPosition(
            degreesToTurretAngle(degrees.getAsDouble()) * TurretConstants.gearRatio / 360.0)))
            .alongWith(Commands.run(() -> setDesiredPosition(degrees.getAsDouble())));
    }

    public Command getTurretPIDCommand(DoubleSupplier degrees, DoubleSupplier radPerSec) {
        return run(() -> {
            // Convert robot angular velocity (rad/s) → degrees/sec
            double degPerSec = Math.toDegrees(radPerSec.getAsDouble());

            // Convert degrees/sec → motor rotations/sec
            double motorRPS = degPerSec / 360.0 * TurretConstants.gearRatio;

            // Command the turret: hold position + apply feedforward
            turretMotor.setControl(positionRequest.withPosition(
                degreesToTurretAngle(degrees.getAsDouble()) * TurretConstants.gearRatio / 360.0)
                .withVelocity(motorRPS));
        }).alongWith(Commands.run(() -> setDesiredPosition(degrees.getAsDouble())));
    }

    // convert angles in (0, 360) to (-180, 180)
    public double normalizeDegrees(double degrees) {
        return (degrees + 180.0) % 360.0 - 180.0;
    }

    public double angleError(double a, double b) {
        double error = a - b;
        error = (error + 180) % 360;
        if (error < 0) error += 360;
        return error - 180;
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
        return MathUtil.clamp(wrappedTarget, TurretConstants.counterclockwiseTurretLimitDegrees, TurretConstants.clockwiseTurretLimitDegrees);
    }

    // get the desired shot angle based on the position of the robot
    // then translates to the turret angle needed to achieve the shot angle based on the rotation of the robot
    public DoubleSupplier getDesiredTurretAngle(Supplier<Pose2d> targetPose, Supplier<Pose2d> drivePose) {
        return () -> {
            Rotation2d desiredShotAngle = Rotation2d.fromRadians(Math.atan2(targetPose.get().getY() - drivePose.get().getY(),
            targetPose.get().getX() - drivePose.get().getX()));
            SmartDashboard.putNumber("shot angle", desiredShotAngle.getDegrees());
            return (desiredShotAngle.getDegrees() - drivePose.get().getRotation().getDegrees());
        };
    }

    public DoubleSupplier getDesiredTurretAngle(DoubleSupplier angle, Supplier<Pose2d> drivePose) {
        return () -> {
            return (angle.getAsDouble() - drivePose.get().getRotation().getDegrees());
        };
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return turretRoutine.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return turretRoutine.dynamic(direction);
    }

    @Override
    public void simulationPeriodic() {
        var talonFXSim = turretMotor.getSimState();

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
        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(TurretConstants.gearRatio));
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(TurretConstants.gearRatio));

        SmartDashboard.putNumber(
            "Turret Motor Volts",
            turretMotor.getSimState().getMotorVoltageMeasure().in(Volts));
    }
}