package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldConstants.ReefBranch;
import frc.robot.util.FieldConstants.ReefFace;
import frc.robot.util.FieldConstants.ReefHeight;
import frc.robot.util.FieldConstants.ReefIndex;

public class AutomatedTeleopCommand extends Command {
    private static enum ControllerButton {
        A,
        X,
        B,
        Y,
        POVUp,
        POVDown,
        POVLeft,
        POVRight
    }

    private static Optional<ReefHeight> controllerButtonToReefHeight(ControllerButton b) {
        switch (b) {
            case POVDown:
                return Optional.of(ReefHeight.L2);
            case POVLeft:
            case POVRight:
                return Optional.of(ReefHeight.L3);
            case POVUp:
                return Optional.of(ReefHeight.L4);
            default:
                return Optional.empty();
        }
    }

    private static Optional<ReefFace> controllerButtonsToReefFace(ControllerButton b1, ControllerButton b2) {
        if (b1 == ControllerButton.A) {
            switch (b2) {
                case A:
                    return Optional.of(ReefFace.E);
                case X:
                    return Optional.of(ReefFace.F);
                case B:
                    return Optional.of(ReefFace.J);
                default:
                    return Optional.empty();
            }
        }
        else if (b1 == ControllerButton.Y) {
            switch (b2) {
                case Y:
                    return Optional.of(ReefFace.H);
                case X:
                    return Optional.of(ReefFace.G);
                case B:
                    return Optional.of(ReefFace.I);
                default:
                    return Optional.empty();
            }
        }

        return Optional.empty();
    }

    private static Optional<ReefIndex> controllerButtonToReefIndex(ControllerButton b) {
        switch (b) {
            case X:
                return Optional.of(ReefIndex.One);
            case B:
                return Optional.of(ReefIndex.Two);
            default:
                return Optional.empty();
        }
    }

    private final ElevatorSubsystem m_elevator;
    private final SwerveSubsystem m_swerve;

    private final CommandPS4Controller m_driverController;
    private final CommandXboxController m_operatorController;

    private final List<ControllerButton> m_queuedBranch;
    private Optional<Command> m_queuedCommand;

    public AutomatedTeleopCommand(
        ElevatorSubsystem elevator,
        SwerveSubsystem swerve,
        CommandPS4Controller driverController,
        CommandXboxController operatorController
    ) {
        m_elevator = elevator;
        m_swerve = swerve;
        addRequirements(m_elevator, m_swerve);

        m_driverController = driverController;
        m_operatorController = operatorController;

        m_queuedBranch = new ArrayList<>(4);
        m_queuedCommand = Optional.empty();
    }

    @Override
    public void execute() {
        updateQueued();

        if (m_queuedCommand.isPresent() && m_driverController.options().getAsBoolean()) {
            m_queuedCommand.get().alongWith(
                ControllerRumbleCommand.makeLinearDecay(1, RumbleType.kBothRumble, 0.5, new GenericHID[] {
                    m_driverController.getHID(), m_operatorController.getHID()
                })
            ).schedule();
            m_queuedCommand = Optional.empty();
            m_operatorController.setRumble(RumbleType.kBothRumble, 0.0);
            m_driverController.setRumble(RumbleType.kBothRumble, 0.0);
        }

        if (m_operatorController.leftBumper().getAsBoolean() && m_operatorController.rightBumper().getAsBoolean()) {
            ControllerRumbleCommand.makeLinearDecay(0.5, RumbleType.kBothRumble, 0.2, new GenericHID[] {
                m_operatorController.getHID()
            }).schedule();
            m_queuedBranch.clear();
        }

        if (m_queuedCommand.isPresent()) {
            m_operatorController.setRumble(RumbleType.kBothRumble, 0.5);
            m_driverController.setRumble(RumbleType.kBothRumble, 0.5);
        }
    }

    private void updateQueued() {
        if (m_operatorController.povUp().getAsBoolean())
            m_queuedBranch.add(ControllerButton.POVUp);
        if (m_operatorController.povDown().getAsBoolean())
            m_queuedBranch.add(ControllerButton.POVDown);
        else if (m_operatorController.povLeft().getAsBoolean())
            m_queuedBranch.add(ControllerButton.POVLeft);
        else if (m_operatorController.povRight().getAsBoolean())
            m_queuedBranch.add(ControllerButton.POVRight);
        else if (m_operatorController.a().getAsBoolean())
            m_queuedBranch.add(ControllerButton.A);
        else if (m_operatorController.b().getAsBoolean())
            m_queuedBranch.add(ControllerButton.B);
        else if (m_operatorController.x().getAsBoolean())
            m_queuedBranch.add(ControllerButton.X);
        else if (m_operatorController.y().getAsBoolean())
            m_queuedBranch.add(ControllerButton.Y);

        if (m_queuedBranch.size() == 4) {
            Optional<ReefHeight> height = controllerButtonToReefHeight(m_queuedBranch.get(0));
            Optional<ReefFace> face = controllerButtonsToReefFace(m_queuedBranch.get(1), m_queuedBranch.get(2));
            Optional<ReefIndex> index = controllerButtonToReefIndex(m_queuedBranch.get(3));
            if (height.isPresent() || face.isPresent() || index.isPresent()) {
                ReefBranch branch = new ReefBranch(
                    height.get(),
                    face.get(),
                    index.get()
                );

                m_queuedCommand = Optional.of(branch.makeScoreCommand(m_swerve, m_elevator));
            }
            else {
                DriverStation.reportWarning("Invalid reef branch combo!", true);
            }

            m_queuedBranch.clear();
        }
    }
}
