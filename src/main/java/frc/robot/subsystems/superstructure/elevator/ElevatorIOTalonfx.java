package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Conversions;

public class ElevatorIOTalonfx implements ElevatorIO {
    protected final TalonFX m_motor;

    private final boolean m_useFOC;

    private final MotionMagicVoltage m_positionVoltage;
    private final VoltageOut m_voltage;

    public ElevatorIOTalonfx(int motorPort, boolean useFOC) {
        m_motor = new TalonFX(motorPort);

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
        m_motor.getConfigurator().apply(configs);

        m_positionVoltage = new MotionMagicVoltage(0.0);
        m_positionVoltage.EnableFOC = m_useFOC;

        m_voltage = new VoltageOut(0.0);
        m_voltage.EnableFOC = m_useFOC;
    }

    public void setPosition(double positionMeters) {
        m_motor.setControl(m_positionVoltage.withPosition(Conversions.elevatorPositionToRotations(positionMeters)));
    }

    public void setVoltage(double volts) {
        m_motor.setControl(m_voltage.withOutput(volts));
    }
}
