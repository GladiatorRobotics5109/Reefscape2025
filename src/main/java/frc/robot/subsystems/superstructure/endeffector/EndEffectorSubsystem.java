package frc.robot.subsystems.superstructure.endeffector;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
    private final EndEffectorIOInputsAutoLogged m_inputs;
    private final EndEffectorIO m_io;

    // Detects the leading edge of coral entering the box
    // private final LoggedDigitalInput m_coralSensorLeading;
    // Detects coral at the center of the box
    // private final LoggedDigitalInput m_coralSensor;

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

        // m_coralSensor = new LoggedDigitalInput(
        //     EndEffectorConstants.kLogPath + "/CoralSensor",
        //     EndEffectorConstants.kCoralSensorPort,
        //     Constants.kCurrentMode
        // );
        // m_coralSensorLeading = new LoggedDigitalInput(
        //     EndEffectorConstants.kLogPath + "/CoralSensorLeading",
        //     EndEffectorConstants.kCoralSensorLeadingPort,
        //     Constants.kCurrentMode
        // );

        m_inputs = new EndEffectorIOInputsAutoLogged();
    }

    //    private final SparkMax m_leftMotor = new SparkMax(0, MotorType.kBrushless);
    //    private final SparkMax m_rightMotor = new SparkMax(0, MotorType.kBrushless);

    public void setVoltage(double leftVolts, double rightVolts) {
        m_io.setVoltage(leftVolts, rightVolts);
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
        // return m_coralSensor.get();
        return false;
    }

    public boolean hasLeadingEdgeCoral() {
        // return m_coralSensorLeading.get();
        return false;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            stop();
        }
    }
}
