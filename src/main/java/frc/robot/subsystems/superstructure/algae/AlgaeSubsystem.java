package frc.robot.subsystems.superstructure.algae;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import org.littletonrobotics.junction.Logger;

public class AlgaeSubsystem extends SubsystemBase {
    private final AlgaeIO m_io;
    private final AlgaeIOInputsAutoLogged m_inputs;

    private Rotation2d m_desiredPosition;
    private boolean m_hasDesiredPosition;

    private final PIDController m_pid;
    private final ArmFeedforward m_feedforward;

    public AlgaeSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new AlgaeIO() {};

                break;
            case SIM:
            default:
                m_io = new AlgaeIO() {};

                break;
        }

        m_inputs = new AlgaeIOInputsAutoLogged();

        m_pid = AlgaeConstants.kPID.getPIDController();
        m_feedforward = AlgaeConstants.kFeedforward.getArmFeedforward();
    }

    public void setDesiredPosition(Rotation2d position) {
        m_hasDesiredPosition = true;
        m_desiredPosition = Rotation2d.fromRadians(
            MathUtil.clamp(
                position.getRadians(),
                AlgaeConstants.kMinPosition.getRadians(),
                AlgaeConstants.kMaxPosition.getRadians()
            )
        );
    }

    public void toRemove() {
        setDesiredPosition(AlgaeConstants.kRemovePosition);
    }

    public void toStow() {
        setDesiredPosition(AlgaeConstants.kStowPosition);
    }

    public void setVoltage(double volts) {
        m_hasDesiredPosition = false;
        m_io.setVoltage(volts);
    }

    public void stop() {
        setVoltage(0.0);
    }

    public Rotation2d getCurrentPosition() { return Rotation2d.fromRadians(m_inputs.positionRad); }

    public Rotation2d getDesiredPosition() { return m_desiredPosition; }

    public boolean hasDesiredPosition() {
        return m_hasDesiredPosition;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(AlgaeConstants.kLogPath, m_inputs);

        if (DriverStation.isDisabled()) {
            stop();
        }
        else if (m_hasDesiredPosition) {
            double pidOut = m_pid.calculate(m_inputs.positionRad, m_desiredPosition.getRadians());
            double feedforward = m_feedforward.calculate(m_inputs.positionRad, 0.0);

            m_io.setVoltage(pidOut + feedforward);
        }
    }
}
