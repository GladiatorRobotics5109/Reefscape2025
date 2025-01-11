// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.FieldUtil.ReefBranches;

public class RobotContainer {
    private final SwerveSubsystem m_swerve;
    private final VisionSubsystem m_vision;
    private final ElevatorSubsystem m_elevator;

    private final CommandXboxController m_driverController;
    // private final CommandPS5Controller m_driverController;
    // private final CommandGenericHID m_keyboard;

    public RobotContainer() {
        m_swerve = new SwerveSubsystem();
        m_vision = new VisionSubsystem();
        m_elevator = new ElevatorSubsystem();
        RobotState.init(m_swerve, m_vision, m_elevator);

        m_driverController = new CommandXboxController(0);
        // m_driverController = new CommandPS5Controller(0);
        // m_keyboard = new CommandGenericHID(0);

        configureBindings();
    }

    private void configureBindings() {
        // m_swerve.setDefaultCommand(SwerveCommandFactory.makeTeleop(m_swerve, m_driverController));
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
        ReefBranches branch = ReefBranches.L2E1;
        return Commands.sequence(
            Commands.waitSeconds(2),
            branch.makeScoreCommand(m_swerve, m_elevator)
        );
        // return Commands.sequence(
        // SwerveControllerFactory.makeSysIdDrive(m_swerve, 0).quasistatic(SysIdRoutine.Direction.kForward),
        // Commands.waitSeconds(2),
        // SwerveControllerFactory.makeSysIdDrive(m_swerve, 0).quasistatic(SysIdRoutine.Direction.kReverse),
        // Commands.waitSeconds(2),
        // SwerveControllerFactory.makeSysIdDrive(m_swerve, 0).dynamic(SysIdRoutine.Direction.kForward),
        // Commands.waitSeconds(2),
        // SwerveControllerFactory.makeSysIdDrive(m_swerve, 0).dynamic(SysIdRoutine.Direction.kReverse),
        // Commands.print("-- SysIdCompleted! -- ")
        // );
    }
}
