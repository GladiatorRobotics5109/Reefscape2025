package frc.robot.subsystems.superstructure.endeffector;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.util.Conversions;

public class EndEffectorIOSparkMax implements EndEffectorIO {
    private final SparkMax m_left;
    private final SparkMax m_right;

    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

    public EndEffectorIOSparkMax(int leftPort, int rightPort) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit((int)EndEffectorConstants.kSupplyCurrentLimit);
        config.encoder.positionConversionFactor(1 / EndEffectorConstants.kGearRatio);

        config.idleMode(IdleMode.kBrake);
        config.inverted(EndEffectorConstants.kInvertMotor);

        m_left = new SparkMax(leftPort, MotorType.kBrushless);
        m_right = new SparkMax(rightPort, MotorType.kBrushless);

        m_left.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_right.configure(
            config.inverted(!EndEffectorConstants.kInvertMotor),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        m_leftEncoder = m_left.getEncoder();
        m_leftEncoder.setPosition(0.0);

        m_rightEncoder = m_right.getEncoder();
        m_rightEncoder.setPosition(0.0);
    }

    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.leftPositionRad = Conversions.rotationsToRadians(m_leftEncoder.getPosition());
        inputs.leftVelocityRadPerSec = Conversions.rotationsPerMinuteToRadiansPerSecond(m_leftEncoder.getVelocity());

        inputs.leftTempCelsius = m_left.getMotorTemperature();
        inputs.leftAppliedVolts = m_left.getAppliedOutput() * m_left.getBusVoltage();
        inputs.leftSupplyVoltage = m_left.getBusVoltage();
        inputs.leftStatorCurrentAmps = m_left.getOutputCurrent();
        // v_supply * I_supply = v_stator * I_stator
        // I_supply = (v_stator * I_stator) / v_supply
        // TODO: verify this
        inputs.leftSupplyCurrentAmps = (m_left.getOutputCurrent() * inputs.leftAppliedVolts) / inputs.leftSupplyVoltage;

        inputs.rightPositionRad = Conversions.rotationsToRadians(m_rightEncoder.getPosition());
        inputs.rightVelocityRadPerSec = Conversions.rotationsPerMinuteToRadiansPerSecond(m_rightEncoder.getVelocity());

        inputs.rightTempCelsius = m_right.getMotorTemperature();
        inputs.rightAppliedVolts = m_right.getAppliedOutput() * m_right.getBusVoltage();
        inputs.rightSupplyVoltage = m_right.getBusVoltage();
        inputs.rightStatorCurrentAmps = m_right.getOutputCurrent();
        // v_supply * I_supply = v_stator * I_stator
        // I_supply = (v_stator * I_stator) / v_supply
        // TODO: verify this
        inputs.rightSupplyCurrentAmps = (m_right.getOutputCurrent() * inputs.rightAppliedVolts)
            / inputs.rightSupplyVoltage;
    }

    public void setVoltage(double leftVolts, double rightVolts) {
        m_left.setVoltage(leftVolts);
        m_right.setVoltage(rightVolts);
    }
}
