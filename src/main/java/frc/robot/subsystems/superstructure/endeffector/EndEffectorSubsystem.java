package frc.robot.subsystems.superstructure.endeffector;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggeddigitalinput.LoggedDigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
    private final EndEffectorIOInputsAutoLogged m_inputs;
    private final EndEffectorIO m_io;

    // Detects the leading edge of coral entering the box
    private final LoggedDigitalInput m_coralSensorLeading;
    // Detects coral at the center of the box
    private final LoggedDigitalInput m_coralSensor;

    public EndEffectorSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new EndEffectorIOSparkMax(EndEffectorConstants.kLeftPort, EndEffectorConstants.kRightPort);

                break;
            case SIM:
            default:
                m_io = new EndEffectorIO() {};

                break;
        }

        m_coralSensor = new LoggedDigitalInput(
            EndEffectorConstants.kLogPath + "/CoralSensor",
            EndEffectorConstants.kCoralSensorPort,
            Constants.kCurrentMode
        );
        m_coralSensorLeading = new LoggedDigitalInput(
            EndEffectorConstants.kLogPath + "/CoralSensorLeading",
            EndEffectorConstants.kCoralSensorLeadingPort,
            Constants.kCurrentMode
        );

        m_inputs = new EndEffectorIOInputsAutoLogged();
    }

    public void setVoltage(double leftVolts, double rightVolts) {
    }

    public void setVoltage(double volts) {
        setVoltage(volts, volts);
    }

    //outtake power setter
    public void setScore() {
        setVoltage(EndEffectorConstants.kScoreVoltage);
    }

    public void setIntake() {
        setVoltage(EndEffectorConstants.kIntakeVoltage);
    }

    public void setIntakeSlow() {
        setVoltage(EndEffectorConstants.kIntakeSlowVoltage);
    }

    public void stop() {
        setVoltage(0);
    }

    public boolean hasCoral() {
        return m_coralSensor.get();
    }

    public boolean hasLeadingEdgeCoral() {
        return m_coralSensorLeading.get();
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            stop();
        }
    }
}
