package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldConstants.ReefBranch;
import frc.robot.util.FieldConstants.ReefFace;
import frc.robot.util.FieldConstants.ReefHeight;
import frc.robot.util.FieldConstants.ReefIndex;

public class AutomatedTeleopControllerListenerCommand extends Command {
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

    private final CommandXboxController m_driverController;
    private final CommandXboxController m_operatorController;

    private final List<ControllerButton> m_buttonQueue;
    private Optional<ReefBranch> m_queuedBranch;

    public AutomatedTeleopControllerListenerCommand(
        ElevatorSubsystem elevator,
        SwerveSubsystem swerve,
        CommandXboxController driverController,
        CommandXboxController operatorController
    ) {
        m_elevator = elevator;
        m_swerve = swerve;

        m_driverController = driverController;
        m_operatorController = operatorController;

        m_buttonQueue = new ArrayList<>();
        m_queuedBranch = Optional.empty();

        m_operatorController.povUp().onTrue(Commands.runOnce(() -> m_buttonQueue.add(ControllerButton.POVUp)));
        m_operatorController.povDown().onTrue(Commands.runOnce(() -> m_buttonQueue.add(ControllerButton.POVDown)));
        m_operatorController.povLeft().onTrue(Commands.runOnce(() -> m_buttonQueue.add(ControllerButton.POVLeft)));
        m_operatorController.povRight().onTrue(Commands.runOnce(() -> m_buttonQueue.add(ControllerButton.POVRight)));
        m_operatorController.a().onTrue(Commands.runOnce(() -> m_buttonQueue.add(ControllerButton.A)));
        m_operatorController.b().onTrue(Commands.runOnce(() -> m_buttonQueue.add(ControllerButton.B)));
        m_operatorController.x().onTrue(Commands.runOnce(() -> m_buttonQueue.add(ControllerButton.X)));
        m_operatorController.y().onTrue(Commands.runOnce(() -> m_buttonQueue.add(ControllerButton.Y)));

        m_operatorController.leftBumper().and(m_operatorController.rightBumper()).onTrue(
            Commands.parallel(
                Commands.runOnce(() -> m_buttonQueue.clear()),
                ControllerRumbleCommand.makeLinearDecay(0.5, RumbleType.kBothRumble, 0.2, new GenericHID[] {
                    m_operatorController.getHID()
                })
            )
        );

        m_driverController.rightBumper().onTrue(Commands.runOnce(() -> {
            if (m_queuedBranch.isEmpty())
                return;
            m_queuedBranch.get().makeScoreCommand(swerve, elevator).alongWith(
                ControllerRumbleCommand.makeLinearDecay(1, RumbleType.kBothRumble, 0.5, new GenericHID[] {
                    m_driverController.getHID(), m_operatorController.getHID()
            })
            ).beforeStarting(() -> {
                m_queuedBranch = Optional.empty();
                m_driverController.setRumble(RumbleType.kBothRumble, 0.0);
                m_operatorController.setRumble(RumbleType.kBothRumble, 0.0);
            }).schedule();
        }));

        setName("AutomatedTeleopCommand");
    }

    @Override
    public void execute() {
        if (m_buttonQueue.size() == 4 && m_queuedBranch.isEmpty()) {
            Optional<ReefHeight> height = controllerButtonToReefHeight(m_buttonQueue.get(0));
            Optional<ReefFace> face = controllerButtonsToReefFace(m_buttonQueue.get(1), m_buttonQueue.get(2));
            Optional<ReefIndex> index = controllerButtonToReefIndex(m_buttonQueue.get(3));
            if (height.isPresent() && face.isPresent() && index.isPresent()) {
                m_operatorController.setRumble(RumbleType.kBothRumble, 1.0);
                m_driverController.setRumble(RumbleType.kBothRumble, 1.0);

                m_queuedBranch = Optional.of(
                    new ReefBranch(
                        height.get(),
                        face.get(),
                        index.get()
                    )
                );

                m_buttonQueue.clear();
            }
            else {
                DriverStation.reportWarning("Invalid reef branch combo!", true);
                m_buttonQueue.clear();
            }
        }

        if (m_queuedBranch.isPresent()) {
            System.out.println("Ready to autoscore!");
            Logger.recordOutput("QueuedBranch", m_queuedBranch.get().getBranchPosition());
        }

        System.out.print("Button Queue: ");
        for (ControllerButton controllerButton : m_buttonQueue) {
            System.out.print(controllerButton);
        }
        System.out.println();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}