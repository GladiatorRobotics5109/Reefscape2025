package frc.robot.subsystems.superstructure.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Conversions;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkMax m_motor;
    private final SparkMax m_follower;

    private final RelativeEncoder m_encoder;

    public ElevatorIOSparkMax(int motorPort, int followerPort) {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit((int)ElevatorConstants.kSupplyCurrentLimit);
        motorConfig.softLimit.forwardSoftLimit(Conversions.radiansToRotations(ElevatorConstants.kForwardSoftLimitRad));
        motorConfig.softLimit.forwardSoftLimitEnabled(true);
        motorConfig.softLimit.reverseSoftLimit(Conversions.radiansToRotations(ElevatorConstants.kReverseSoftLimitRad));
        motorConfig.softLimit.reverseSoftLimitEnabled(true);

        motorConfig.encoder.positionConversionFactor(1 / ElevatorConstants.kGearRatio);

        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.inverted(ElevatorConstants.kInvertMotor);

        m_motor = new SparkMax(motorPort, MotorType.kBrushless);
        m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_follower = new SparkMax(followerPort, MotorType.kBrushless);
        SparkMaxConfig followerConfig = motorConfig;
        followerConfig.follow(m_motor, true);
        m_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_motor.getEncoder();
        m_encoder.setPosition(0.0);
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRad = Conversions.rotationsToRadians(m_encoder.getPosition());
        inputs.velocityRadPerSec = Conversions.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity());
        inputs.accelerationRadPerSecPerSec = 0.0;

        inputs.motorTempCelsius = m_motor.getMotorTemperature();
        inputs.motorAppliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.motorSupplyVoltage = m_motor.getBusVoltage();
        inputs.motorStatorCurrentAmps = m_motor.getOutputCurrent();
        // v_supply * I_supply = v_stator * I_stator
        // I_supply = (v_stator * I_stator) / v_supply
        // TODO: verify this
        inputs.motorSupplyCurrentAmps = (m_motor.getOutputCurrent() * inputs.motorAppliedVolts)
            / inputs.motorSupplyVoltage;

        inputs.followerMotorTempCelsius = m_follower.getMotorTemperature();
        inputs.followerMotorAppliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.followerMotorSupplyVoltage = m_motor.getBusVoltage();
        inputs.followerMotorStatorCurrentAmps = m_motor.getOutputCurrent();
        inputs.followerMotorSupplyCurrentAmps = (m_motor.getOutputCurrent() * inputs.motorAppliedVolts)
            / inputs.motorSupplyVoltage;

        inputs.motorSoftUpLimited = false;
        inputs.motorSoftDownLimited = false;
    }

    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }
}
