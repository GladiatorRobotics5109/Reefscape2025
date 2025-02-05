package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Conversions;

public class ElevatorIOTalonFX implements ElevatorIO {
    protected final TalonFX m_motor;
    protected final TalonFX m_followerMotor;

    private final boolean m_useFOC;

    private final MotionMagicVoltage m_positionVoltage;
    private final VoltageOut m_voltage;
    private final Follower m_follower;

    private final StatusSignal<Angle> m_signalPosition;
    private final StatusSignal<AngularVelocity> m_signalVelocity;
    private final StatusSignal<AngularAcceleration> m_signalAcceleration;
    private final StatusSignal<Temperature> m_signalTemp;
    private final StatusSignal<Voltage> m_signalAppliedVoltage;
    private final StatusSignal<Voltage> m_signalSupplyVoltage;
    private final StatusSignal<Current> m_signalStatorCurrent;
    private final StatusSignal<Current> m_signalSupplyCurrent;
    private final StatusSignal<Boolean> m_signalSoftUpLimit;
    private final StatusSignal<Boolean> m_signalSoftDownLimit;

    private final StatusSignal<Temperature> m_signalFollowerTemp;
    private final StatusSignal<Voltage> m_signalFollowerAppliedVoltage;
    private final StatusSignal<Voltage> m_signalFollowerSupplyVoltage;
    private final StatusSignal<Current> m_signalFollowerStatorCurrent;
    private final StatusSignal<Current> m_signalFollowerSupplyCurrent;

    public ElevatorIOTalonFX(int motorPort, int followerPort, boolean useFOC) {
        m_motor = new TalonFX(motorPort);
        m_followerMotor = new TalonFX(followerPort);

        m_useFOC = useFOC;

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kStatorCurrentLimit;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.kSupplyCurrentLimit;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;

        configs.Feedback.SensorToMechanismRatio = ElevatorConstants.kGearRatio;

        configs.Slot0.kP = ElevatorConstants.kPID.kp();
        configs.Slot0.kI = ElevatorConstants.kPID.ki();
        configs.Slot0.kD = ElevatorConstants.kPID.kd();
        configs.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        configs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        configs.Slot0.kG = ElevatorConstants.kFeedForward.kg();
        configs.Slot0.kS = ElevatorConstants.kFeedForward.ks();
        configs.Slot0.kV = ElevatorConstants.kFeedForward.kv();
        configs.Slot0.kA = ElevatorConstants.kFeedForward.ka();

        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.kForwardSoftLimit;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.kReverseSoftLimit;

        configs.MotorOutput.Inverted = ElevatorConstants.kInvertMotor
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

        configs.MotionMagic.MotionMagicCruiseVelocity = Conversions.radiansToRotations(
            ElevatorConstants.kElevatorCruiseVelocityRadPerSec
        );
        configs.MotionMagic.MotionMagicAcceleration = Conversions.radiansToRotations(
            ElevatorConstants.kElevatorAccelerationRadPerSecPerSec
        );

        m_motor.getConfigurator().apply(configs);
        m_followerMotor.getConfigurator().apply(configs);

        m_motor.setPosition(0);
        m_followerMotor.setPosition(0);

        m_positionVoltage = new MotionMagicVoltage(0.0);
        m_positionVoltage.EnableFOC = m_useFOC;

        m_voltage = new VoltageOut(0.0);
        m_voltage.EnableFOC = m_useFOC;

        m_follower = new Follower(m_motor.getDeviceID(), true);
        m_followerMotor.setControl(m_follower);

        m_signalPosition = m_motor.getPosition();
        m_signalVelocity = m_motor.getVelocity();
        m_signalAcceleration = m_motor.getAcceleration();
        m_signalTemp = m_motor.getDeviceTemp();
        m_signalAppliedVoltage = m_motor.getMotorVoltage();
        m_signalSupplyVoltage = m_motor.getSupplyVoltage();
        m_signalStatorCurrent = m_motor.getStatorCurrent();
        m_signalSupplyCurrent = m_motor.getSupplyCurrent();
        m_signalSoftUpLimit = m_motor.getFault_ForwardSoftLimit();
        m_signalSoftDownLimit = m_motor.getFault_ReverseSoftLimit();

        m_signalFollowerTemp = m_followerMotor.getDeviceTemp();
        m_signalFollowerAppliedVoltage = m_followerMotor.getMotorVoltage();
        m_signalFollowerSupplyVoltage = m_followerMotor.getSupplyVoltage();
        m_signalFollowerStatorCurrent = m_followerMotor.getStatorCurrent();
        m_signalFollowerSupplyCurrent = m_followerMotor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            m_signalPosition,
            m_signalVelocity,
            m_signalAcceleration,
            m_signalTemp,
            m_signalAppliedVoltage,
            m_signalSupplyVoltage,
            m_signalStatorCurrent,
            m_signalSupplyCurrent,
            m_signalSoftUpLimit,
            m_signalSoftDownLimit,

            m_signalFollowerTemp,
            m_signalFollowerAppliedVoltage,
            m_signalFollowerSupplyVoltage,
            m_signalFollowerStatorCurrent,
            m_signalFollowerSupplyCurrent
        );

        inputs.positionRad = m_signalPosition.getValue().in(Units.Radians);
        inputs.velocityRadPerSec = m_signalVelocity.getValue().in(Units.RadiansPerSecond);
        inputs.accelerationRadPerSecPerSec = m_signalAcceleration.getValue().in(Units.RadiansPerSecondPerSecond);
        inputs.motorTempCelsius = m_signalTemp.getValue().in(Units.Celsius);
        inputs.motorAppliedVolts = m_signalAppliedVoltage.getValue().in(Units.Volts);
        inputs.motorSupplyVoltage = m_signalSupplyVoltage.getValue().in(Units.Volts);
        inputs.motorStatorCurrentAmps = m_signalStatorCurrent.getValue().in(Units.Amps);
        inputs.motorSupplyCurrentAmps = m_signalSupplyCurrent.getValue().in(Units.Amps);
        inputs.motorSoftUpLimited = m_signalSoftUpLimit.getValue();
        inputs.motorSoftDownLimited = m_signalSoftDownLimit.getValue();

        inputs.followerMotorTempCelsius = m_signalFollowerTemp.getValue().in(Units.Celsius);
        inputs.followerMotorAppliedVolts = m_signalFollowerAppliedVoltage.getValue().in(Units.Volts);
        inputs.followerMotorSupplyVoltage = m_signalFollowerSupplyVoltage.getValue().in(Units.Volts);
        inputs.followerMotorStatorCurrentAmps = m_signalFollowerStatorCurrent.getValue().in(Units.Amps);
        inputs.followerMotorSupplyCurrentAmps = m_signalFollowerSupplyCurrent.getValue().in(Units.Amps);
    }

    @Override
    public void setPosition(double positionRot) {
        m_motor.setControl(m_positionVoltage.withPosition(positionRot));
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setControl(m_voltage.withOutput(volts));
    }
}
