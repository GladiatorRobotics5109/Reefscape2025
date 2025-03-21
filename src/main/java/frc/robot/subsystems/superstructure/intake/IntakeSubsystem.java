package frc.robot.subsystems.superstructure.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO m_io;
    private final IntakeIOInputsAutoLogged m_inputs;

    public IntakeSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new IntakeIOSparkMax(IntakeConstants.kMotorPort);

                break;
            case SIM:
            default:
                m_io = new IntakeIO() {};
        }

        m_inputs = new IntakeIOInputsAutoLogged();
    }

    public void setVoltage(double volts) {
        m_io.setVoltage(volts);
    }

    public void stop() {
        setVoltage(0.0);
    }

    public void intake() {
        setVoltage(IntakeConstants.kIntakeVoltage);
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(IntakeConstants.kLogPath, m_inputs);

        if (DriverStation.isDisabled()) {
            stop();
        }
    }
}
