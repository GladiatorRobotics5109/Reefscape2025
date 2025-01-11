package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Conversions;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO m_io;
    private final ElevatorIOInputsAutoLogged m_inputs;

    public ElevatorSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new ElevatorIOTalonfx(ElevatorConstants.kMotorPort, ElevatorConstants.kUseFOC);

                break;
            default:
                m_io = new ElevatorIO() {};
        }

        m_inputs = new ElevatorIOInputsAutoLogged();
    }

    public void setPosition(double positionMeters) {
        m_io.setPosition(positionMeters);
    }

    public void setVoltage(double volts) {
        m_io.setVoltage(volts);
    }

    public double getPosition() {
        return Conversions.elevatorRotationsToElevatorPosition(m_inputs.positionRad);
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(ElevatorConstants.kLogPath, m_inputs);

        if (DriverStation.isDisabled()) {
            m_io.setVoltage(0);
        }
    }
}
