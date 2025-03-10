package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
    public static double getReefHeightMeters(ReefHeight height) {
        return switch (height) {
            case L1 -> ElevatorConstants.kL1HeightMeters;
            case L2 -> ElevatorConstants.kL2HeightMeters;
            case L3 -> ElevatorConstants.kL3HeightMeters;
            case L4 -> ElevatorConstants.kL4HeightMeters;
        };
    }

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
                m_io = new ElevatorIOSparkMax(ElevatorConstants.kMotorPort, ElevatorConstants.kFollowerPort);

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
            new LoggedMechanismLigament2d(
                "Elevator",
                ElevatorConstants.kElevatorBaseHeightMeters,
                90,
                6,
                new Color8Bit(0, 0, 0)
            )
        );
        m_mechEndEffector = m_mechElevator.append(
            new LoggedMechanismLigament2d(
                "EndEffector",
                ElevatorConstants.kEndEffectorHeightMeters - ElevatorConstants.kElevatorBaseHeightMeters,
                0,
                3,
                new Color8Bit(255, 0, 0)
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

    public void toHome() {
        m_hasDesiredPosition = true;
        m_desiredPositionMeters = 0.0;
        if (m_useMotorPID)
            m_io.setPosition(m_desiredPositionMeters);
    }

    public void stop() {
        setVoltage(0.0);
    }

    public void setVoltage(double volts) {
        m_hasDesiredPosition = false;
        m_io.setVoltage(volts);
    }

    public void setDesiredPositionElevator(double positionMeters) {
        m_hasDesiredPosition = true;
        m_desiredPositionMeters = MathUtil.clamp(positionMeters, 0.0, ElevatorConstants.kElevatorMaxPositionMeters);
        if (m_useMotorPID)
            m_io.setPosition(Conversions.elevatorMetersToElevatorRadians(m_desiredPositionMeters));
    }

    /**
     * Sets the desired height of the end effector measured from the floor
     */
    public void setDesiredPositionEndEffector(double positionMeters) {
        setDesiredPositionElevator(Conversions.endEffectorMetersToElevatorMeters(positionMeters));
    }

    public void setDesiredPositionEndEffector(ReefHeight height) {
        setDesiredPositionEndEffector(getReefHeightMeters(height));
    }

    public double getCurrentPositionElevator() {
        return Conversions.elevatorRadiansToElevatorMeters(m_inputs.positionRad);
    }

    public double getCurrentPositionEndEffector() {
        return Conversions.endEffectorMetersToElevatorMeters(getCurrentPositionElevator());
    }

    public double getCurrentVelocity() {
        return Conversions.elevatorRadiansToElevatorMeters(m_inputs.velocityRadPerSec);
    }

    public double getDesiredPositionElevator() { return m_desiredPositionMeters; }

    public double getCurrentPositionRad() { return m_inputs.positionRad; }

    public boolean atDesiredPosition() {
        return MathUtil.isNear(
            getDesiredPositionElevator(),
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
        else if (m_hasDesiredPosition && !m_useMotorPID) {
            double pidOut = m_pid.calculate(
                m_inputs.positionRad,
                Conversions.elevatorMetersToElevatorRadians(m_desiredPositionMeters)
            );

            TrapezoidProfile.State desiredState = m_pid.getSetpoint();

            m_io.setVoltage(
                pidOut + m_feedforward.calculate(desiredState.velocity)
            );

            Logger.recordOutput(
                ElevatorConstants.kLogPath + "/MotionProfile/DesiredPositionRad",
                desiredState.position
            );
            Logger.recordOutput(
                ElevatorConstants.kLogPath + "/MotionProfile/DesiredVelocityRadPerSec",
                desiredState.velocity
            );
        }

        // Update mechanism
        m_mechElevator.setLength(getCurrentPositionElevator());
        Logger.recordOutput(ElevatorConstants.kLogPath + "/Mechanism", m_mech);
    }
}
