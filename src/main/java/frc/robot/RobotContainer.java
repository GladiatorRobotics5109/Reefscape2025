// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutomatedTeleopControllerListenerCommand;
import frc.robot.subsystems.superstructure.elevator.ElevatorCommandFactory;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveCommandFactory;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.Paths;
import frc.robot.util.FieldConstants.ReefHeight;
import frc.robot.util.FieldConstants.ReefBranch;

public class RobotContainer {
    private final SwerveSubsystem m_swerve;
    private final VisionSubsystem m_vision;
    private final ElevatorSubsystem m_elevator;

    private final CommandXboxController m_driverController;
    // private final CommandPS4Controller m_driverController;
    // private final CommandGenericHID m_keyboard;

    private final CommandXboxController m_operatorController;

    public RobotContainer() {
        m_swerve = new SwerveSubsystem();
        m_vision = new VisionSubsystem();
        m_elevator = new ElevatorSubsystem();
        RobotState.init(m_swerve, m_vision, m_elevator);

        // m_driverController = new CommandPS4Controller(Constants.DriveTeamConstants.kDriveControllerPort);
        m_driverController = new CommandXboxController(Constants.DriveTeamConstants.kDriveControllerPort);
        m_operatorController = new CommandXboxController(Constants.DriveTeamConstants.kOperatorControllerPort);
        // m_driverController = new CommandPS5Controller(0);
        // m_keyboard = new CommandGenericHID(0);

        Logger.recordOutput("GeneratedPaths", Paths.generatedPaths);

        configureBindings();

        CommandScheduler.getInstance().onCommandInitialize((Command command) -> {
            Logger.recordOutput("CommandLog", "Started: " + command.getName() + "\n");
        });
        CommandScheduler.getInstance().onCommandFinish((Command command) -> {
            Logger.recordOutput("CommandLog", "Finished: " + command.getName() + "\n");
        });
        CommandScheduler.getInstance().onCommandInterrupt((Command command) -> {
            Logger.recordOutput("CommandLog", "Interrupted: " + command.getName() + "\n");
        });
    }

    private void configureBindings() {
        m_swerve.setDefaultCommand(SwerveCommandFactory.makeTeleop(m_swerve, m_driverController));

        // Elevator setpoints
        // m_driverController.cross().onTrue(ElevatorCommandFactory.toPosition(m_elevator, ReefHeight.L1));
        // m_driverController.circle().onTrue(ElevatorCommandFactory.toPosition(m_elevator, ReefHeight.L2));
        // m_driverController.triangle().onTrue(ElevatorCommandFactory.toPosition(m_elevator, ReefHeight.L3));
        // m_driverController.square().onTrue(ElevatorCommandFactory.toPosition(m_elevator, ReefHeight.L4));
        m_driverController.a().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L1));
        m_driverController.b().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L2));
        m_driverController.y().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L3));
        m_driverController.x().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L4));

        // TODO: impliment this
        // L1 - intake
        // R2 - outake
        // Right d-pad - climb
        // Left d-pad - abort climb

        // Keyboard controls
        // m_swerve.setDefaultCommand(
        // SwerveComSmandFactory.makeTeleop(
        // m_swerve,
        // // () -> 0,
        // // () -> 1,
        // () -> m_keyboard.getRawAxis(0),
        // () -> -m_keyboard.getRawAxis(1),
        // () -> m_keyboard.getRawAxis(2),
        // () -> 0
        // )
        // );
    }

    public Command getAutonomousCommand() {
        // return null;
        ReefBranch branch = FieldConstants.ReefBranch.kL4G2;
        return Commands.sequence(
            SwerveCommandFactory.setPosition(m_swerve, () -> new Pose2d(6, 1, Rotation2d.fromDegrees(190))),
            Commands.waitSeconds(4),
            branch.makeScoreCommand(m_swerve, m_elevator)
        );
    }

    public Command getTeleopCommand() {
        return new AutomatedTeleopControllerListenerCommand(
            m_elevator,
            m_swerve,
            m_driverController,
            m_operatorController
        );
    }
}
