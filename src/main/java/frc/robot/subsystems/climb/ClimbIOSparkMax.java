package frc.robot.subsystems.climb;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.Conversions;

public class ClimbIOSparkMax implements ClimbIO {
    private final SparkMax m_motor;

    private final RelativeEncoder m_encoder;

    public ClimbIOSparkMax(int port) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit((int)ClimbConstants.kSupplyCurrentLimit);
        config.encoder.positionConversionFactor(1 / ClimbConstants.kGearRatio);

        config.idleMode(IdleMode.kBrake);
        config.inverted(ClimbConstants.kInvertMotor);

        m_motor = new SparkMax(port, MotorType.kBrushless);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_motor.getEncoder();
        m_encoder.setPosition(ClimbConstants.kStartingPosition.getRotations());
    }

    public void updateInputs(ClimbIOInputs inputs) {
        inputs.positionRad = Conversions.rotationsToRadians(m_encoder.getPosition());
        inputs.velocityRadPerSec = Conversions.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity());

        inputs.tempCelsius = m_motor.getMotorTemperature();
        inputs.appliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.supplyVoltage = m_motor.getBusVoltage();
        inputs.statorCurrentAmps = m_motor.getOutputCurrent();
        // v_supply * I_supply = v_stator * I_stator
        // I_supply = (v_stator * I_stator) / v_supply
        // TODO: verify this
        inputs.supplyCurrentAmps = (m_motor.getOutputCurrent() * inputs.appliedVolts) / inputs.supplyVoltage;
    }

    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }
}
