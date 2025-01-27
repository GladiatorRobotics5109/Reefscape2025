package frc.robot.subsystems.superstructure.elevator;

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

    private double m_desiredPositionMeters;

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
    }

    public void setVoltage(double volts) {
        m_io.setVoltage(volts);
    }

    public void setDesiredPositionElevatorRelative(double positionMeters) {
        m_desiredPositionMeters = positionMeters;
        m_io.setPosition(Conversions.elevatorMetersToElevatorRotations(positionMeters));
    }

    public void setDesiredPositionEndEffectorRelative(double positionMeters) {
        setDesiredPositionElevatorRelative(Conversions.endEffectorMetersToElevatorMeters(positionMeters));
    }

    public void setDesiredPositionEndEffectorRelative(ReefHeight height) {
        setDesiredPositionEndEffectorRelative(height.getHeight());
    }

    public double getCurrentPositionElevatorRelative() {
        return Conversions.elevatorRotationsToElevatorMeters(m_inputs.positionRad);
    }

    public double getCurrentPositionEndEffectorRelative() {
        return Conversions.endEffectorMetersToElevatorMeters(getCurrentPositionElevatorRelative());
    }

    public double getDesiredPositionElevatorRelative() { return m_desiredPositionMeters; }

    public boolean atDesiredPosition() {
        return MathUtil.isNear(
            getDesiredPositionElevatorRelative(),
            getCurrentPositionElevatorRelative(),
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
        m_io.updateSim();
        m_io.updateInputs(m_inputs);
        Logger.processInputs(ElevatorConstants.kLogPath, m_inputs);

        if (DriverStation.isDisabled()) {
            m_io.setVoltage(0);
        }

        // Update mechanism
        m_mechElevator.setLength(getCurrentPositionElevatorRelative());
        Logger.recordOutput(ElevatorConstants.kLogPath + "/Mechanism", m_mech);
    }
}
