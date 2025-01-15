package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldConstants.ReefBranch;
import frc.robot.util.FieldConstants.ReefFace;
import frc.robot.util.FieldConstants.ReefHeight;

public class TeleopController extends Command {
    private final ElevatorSubsystem m_elevator;
    private final SwerveSubsystem m_swerve;

    private final CommandXboxController m_driverController;
    private final CommandXboxController m_operatorController;

    private Optional<ReefHeight> m_queuedHeight;
    private Optional<ReefFace> m_queuedSubFace;
    private Optional<ReefFace> m_queuedFace;
    private Optional<Integer> m_queuedIndex;

    private Optional<ReefBranch> m_queuedBranch;

    public TeleopController(
        ElevatorSubsystem elevator,
        SwerveSubsystem swerve,
        CommandXboxController driverController,
        CommandXboxController operatorController
    ) {
        m_elevator = elevator;
        m_swerve = swerve;

        m_driverController = driverController;
        m_operatorController = operatorController;

        m_queuedHeight = Optional.empty();
        m_queuedSubFace = Optional.empty();
        m_queuedFace = Optional.empty();
        m_queuedIndex = Optional.empty();

        m_queuedBranch = Optional.empty();
    }

    @Override
    public void execute() {
        if (m_operatorController.povDown().getAsBoolean()) {
            m_queuedHeight = Optional.of(ReefHeight.L2);
        }
        else if (m_operatorController.povLeft().getAsBoolean() || m_operatorController.povRight().getAsBoolean()) {
            m_queuedHeight = Optional.of(ReefHeight.L3);
        }
        else if (m_operatorController.povUp().getAsBoolean()) {
            m_queuedHeight = Optional.of(ReefHeight.L4);
        }

        if (m_queuedHeight.isPresent() && m_queuedBranch.isEmpty()) {
            if (m_operatorController.a().getAsBoolean()) {
                m_queuedSubFace = Optional.of(ReefFace.E);
            }
            else if (m_operatorController.y().getAsBoolean()) {
                m_queuedSubFace = Optional.of(ReefFace.H);
            }
        }
        else if (m_queuedSubFace.isPresent() && m_queuedBranch.isEmpty()) {
            if (m_queuedSubFace.get() == ReefFace.E) {
                if (m_operatorController.a().getAsBoolean()) {
                    m_queuedFace = Optional.of(ReefFace.E);
                }
                else if (m_operatorController.x().getAsBoolean()) {
                    m_queuedFace = Optional.of(ReefFace.F);
                }
                else if (m_operatorController.b().getAsBoolean()) {
                    m_queuedFace = Optional.of(ReefFace.J);
                }
            }
            else if (m_queuedSubFace.get() == ReefFace.H) {
                if (m_operatorController.y().getAsBoolean()) {
                    m_queuedFace = Optional.of(ReefFace.H);
                }
                else if (m_operatorController.x().getAsBoolean()) {
                    m_queuedFace = Optional.of(ReefFace.G);
                }
                else if (m_operatorController.b().getAsBoolean()) {
                    m_queuedFace = Optional.of(ReefFace.I);
                }
            }
        }

        if (m_queuedFace.isPresent() && m_driverController.a().getAsBoolean()) {

        }
    }
}
