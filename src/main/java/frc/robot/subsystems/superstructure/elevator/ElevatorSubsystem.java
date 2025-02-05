package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Conversions;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefConstants.ReefHeight;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO m_io;
    private final ElevatorIOInputsAutoLogged m_inputs;

    private final LoggedMechanism2d m_mech;
    private final LoggedMechanismRoot2d m_mechRoot;
    private final LoggedMechanismLigament2d m_mechElevator;
    private final LoggedMechanismLigament2d m_mechEndEffector;

    private final ProfiledPIDController m_pid;
    private final ElevatorFeedforward m_feedforward;

    private final boolean m_useMotorPID;

    private double m_desiredPositionMeters;
    private boolean m_hasDesiredPosition;

    public ElevatorSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new ElevatorIOTalonFX(
                    ElevatorConstants.kMotorPort,
                    ElevatorConstants.kFollowerPort,
                    ElevatorConstants.kUseFOC
                );

                break;
            case SIM:
                m_io = new ElevatorIOSimTalonFX(
                    ElevatorConstants.kMotorPort,
                    ElevatorConstants.kFollowerPort,
                    ElevatorConstants.kUseFOC
                );

                break;
            default:
                m_io = new ElevatorIO() {};

                break;
        }

        m_inputs = new ElevatorIOInputsAutoLogged();

        m_mech = new LoggedMechanism2d(0, 0);
        m_mechRoot = m_mech.getRoot("Base", 0, ElevatorConstants.kElevatorBaseHeightMeters);
        m_mechElevator = m_mechRoot.append(
            new LoggedMechanismLigament2d("Elevator", ElevatorConstants.kElevatorBaseHeightMeters, 90)
        );
        m_mechEndEffector = m_mechElevator.append(
            new LoggedMechanismLigament2d(
                "EndEffector",
                ElevatorConstants.kEndEffectorHeightMeters - ElevatorConstants.kElevatorBaseHeightMeters,
                0
            )
        );

        m_desiredPositionMeters = 0.0;

        m_pid = new ProfiledPIDController(
            ElevatorConstants.kPID.kp(),
            ElevatorConstants.kPID.ki(),
            ElevatorConstants.kPID.kd(),
            new TrapezoidProfile.Constraints(
                ElevatorConstants.kElevatorCruiseVelocityRadPerSec,
                ElevatorConstants.kElevatorAccelerationRadPerSecPerSec
            )
        );

        m_feedforward = ElevatorConstants.kFeedForward.getElevatorFeedforward();

        m_useMotorPID = ElevatorConstants.kUseMotorPID;
    }

    public void setVoltage(double volts) {
        m_hasDesiredPosition = false;
        m_io.setVoltage(volts);
    }

    public void setDesiredPositionElevator(double positionMeters) {
        m_hasDesiredPosition = true;
        m_desiredPositionMeters = MathUtil.clamp(positionMeters, 0.0, ElevatorConstants.kElevatorMaxPositionMeters);
        m_io.setPosition(Conversions.elevatorMetersToElevatorRotations(m_desiredPositionMeters));
    }

    public void setDesiredPositionEndEffector(double positionMeters) {
        setDesiredPositionElevator(Conversions.endEffectorMetersToElevatorMeters(positionMeters));
    }

    public void setDesiredPositionEndEffector(ReefHeight height) {
        setDesiredPositionEndEffector(height.getHeight());
    }

    public double getCurrentPositionElevator() {
        return Conversions.elevatorRotationsToElevatorMeters(m_inputs.positionRad);
    }

    public double getCurrentPositionEndEffector() {
        return Conversions.endEffectorMetersToElevatorMeters(getCurrentPositionElevator());
    }

    public double getDesiredPositionElevatorRelative() { return m_desiredPositionMeters; }

    public boolean atDesiredPosition() {
        return MathUtil.isNear(
            getDesiredPositionElevatorRelative(),
            getCurrentPositionElevator(),
            ElevatorConstants.kPositionToleranceMeters
        );
    }

    public boolean canAutoExtend() {
        return RobotState.getSwervePose().getTranslation().getDistance(
            FieldConstants.ReefConstants.getAllianceReefPos()
        ) < ElevatorConstants.kAutoElevatorExtendRequiredDistanceMeters;
    }

    @Override
    public void periodic() {
        m_io.periodic();
        m_io.updateInputs(m_inputs);
        Logger.processInputs(ElevatorConstants.kLogPath, m_inputs);

        if (DriverStation.isDisabled()) {
            setVoltage(0);
        }
        else if (m_hasDesiredPosition && m_useMotorPID) {
            m_io.setVoltage(
                m_pid.calculate(
                    m_inputs.positionRad,
                    Conversions.elevatorMetersToElevatorRadians(m_desiredPositionMeters)
                ) + m_feedforward.calculate(m_pid.getSetpoint().velocity)
            );
        }

        // Update mechanism
        m_mechElevator.setLength(getCurrentPositionElevator());
        Logger.recordOutput(ElevatorConstants.kLogPath + "/Mechanism", m_mech);
    }
}
