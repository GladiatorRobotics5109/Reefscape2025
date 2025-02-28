package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {
    private final ClimbIO m_io;
    private final ClimbIOInputsAutoLogged m_inputs;

    private PIDController m_pid;
    private boolean m_hasDesiredPosition;
    private double m_desiredPositionRad;

    public ClimbSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new ClimbIOSparkMax(ClimbConstants.kMotorPort);

                break;
            default:
                m_io = new ClimbIO() {};
        }

        m_pid = ClimbConstants.kPID.getPIDController();
        m_desiredPositionRad = ClimbConstants.kStartingPosition.getRadians();
        m_hasDesiredPosition = false;

        m_inputs = new ClimbIOInputsAutoLogged();
    }

    public void setVoltage(double volts) {
        m_io.setVoltage(volts);
        m_hasDesiredPosition = false;
    }

    public void setDesiredPosition(Rotation2d position) {
        m_desiredPositionRad = MathUtil.clamp(
            position.getRadians(),
            ClimbConstants.kMaxPosition.getRadians(),
            ClimbConstants.kMinPosition.getRadians()
        );
        m_hasDesiredPosition = true;
    }

    public void stop() {
        setVoltage(0.0);
    }

    public boolean atDesiredPosition() {
        return m_hasDesiredPosition
            && MathUtil.isNear(m_desiredPositionRad, m_inputs.positionRad, ClimbConstants.kPositionToleranceRad);
    }

    public boolean atClimbPosition() {
        return MathUtil.isNear(
            ClimbConstants.kClimbPosition.getRadians(),
            m_inputs.positionRad,
            ClimbConstants.kPositionToleranceRad
        ) || m_inputs.positionRad < ClimbConstants.kClimbPosition.getRadians();
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(ClimbConstants.kLogPath, m_inputs);

        if (DriverStation.isDisabled()) {
            stop();
        }
        else if (m_hasDesiredPosition) {
            m_io.setVoltage(m_pid.calculate(m_inputs.positionRad, m_desiredPositionRad));
        }
    }
}
