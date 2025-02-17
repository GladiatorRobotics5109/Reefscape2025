// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.subsystems.leds.LEDSubsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.Conversions;
import frc.robot.util.FieldConstants.ReefConstants.ReefHeight;

public class RobotContainer {
    private SwerveSubsystem m_swerve;
    private VisionSubsystem m_vision;
    private ElevatorSubsystem m_elevator;
    private EndEffectorSubsystem m_endEffector;
    private LEDSubsystem m_leds;

    private final CommandXboxController m_driverController;

    private final CommandXboxController m_operatorController;

    public RobotContainer() {
        // m_swerve = new SwerveSubsystem();
        // m_vision = new VisionSubsystem();
        m_elevator = new ElevatorSubsystem();
        // m_endEffector = new EndEffectorSubsystem();
        // m_leds = new LEDSubsystem();
        RobotState.init(m_swerve, m_vision, m_elevator);
        // AutoChooser.init(m_swerve, m_elevator, m_endEffector, m_leds);
        System.out.println(
            "L1 RAD: "
                + Conversions.elevatorMetersToElevatorRadians(
                    Conversions.endEffectorMetersToElevatorMeters(ReefHeight.L1.getHeight())
                )
        );
        System.out.println(
            "L2 RAD: "
                + Conversions.elevatorMetersToElevatorRadians(
                    Conversions.endEffectorMetersToElevatorMeters(ReefHeight.L2.getHeight())
                )
        );
        System.out.println(
            "L3 RAD: "
                + Conversions.elevatorMetersToElevatorRadians(
                    Conversions.endEffectorMetersToElevatorMeters(ReefHeight.L3.getHeight())
                )
        );
        System.out.println(
            "L4 RAD: "
                + Conversions.elevatorMetersToElevatorRadians(
                    Conversions.endEffectorMetersToElevatorMeters(ReefHeight.L4.getHeight())
                )
        );

        m_driverController = new CommandXboxController(Constants.DriveTeamConstants.kDriveControllerPort);
        m_operatorController = new CommandXboxController(Constants.DriveTeamConstants.kOperatorControllerPort);

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
        m_elevator.setDefaultCommand(
            ElevatorCommandFactory.debugControllerAxis(
                m_elevator,
                m_driverController::getRightTriggerAxis,
                m_driverController::getLeftTriggerAxis
            )
        );
        // m_swerve.setDefaultCommand(
        //     SwerveCommandFactory.makeTeleop(m_swerve, m_driverController).onlyWhile(
        //         () -> DriverStation.isTeleop() || DriverStation.isTest()
        //     )
        // );

        // Elevator setpoints
        // m_driverController.cross().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L1));
        // m_driverController.circle().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L2));
        // m_driverController.triangle().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L3));
        // m_driverController.square().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L4));
        m_driverController.a().onTrue(
            ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L1).andThen(
                Commands.waitUntil(m_elevator::atDesiredPosition).andThen(Commands.waitSeconds(0.1))
            )
        );
        m_driverController.b().onTrue(
            ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L2).andThen(
                Commands.waitUntil(m_elevator::atDesiredPosition).andThen(Commands.waitSeconds(0.1))
            )
        );
        m_driverController.y().onTrue(
            // ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L3).andThen(
            //     Commands.waitUntil(m_elevator::atDesiredPosition).andThen(Commands.waitSeconds(2))
            // )
            ElevatorCommandFactory.toElevatorRelativeHeight(m_elevator, () -> 0.0).andThen(
                Commands.waitUntil(m_elevator::atDesiredPosition).andThen(Commands.waitSeconds(0.1))
            )
        );
        m_driverController.x().onTrue(
            ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L4).andThen(
                Commands.waitUntil(m_elevator::atDesiredPosition).andThen(Commands.waitSeconds(0.1))
            )
        );

        // m_driverController.leftBumper().debounce(0.5, DebounceType.kRising).onTrue(
        //     Commands.parallel(
        //         AutoBuilder.makeAutoDecideScoreCommand(ReefHeight.L4, m_swerve, m_elevator, m_endEffector, m_leds),
        //         ControllerRumbleCommand.linearDecayCommand(
        //             1.0,
        //             RumbleType.kBothRumble,
        //             1.5,
        //             new GenericHID[] { m_driverController.getHID(), m_operatorController.getHID() }
        //         ),
        //         LEDCommandFactory.goodThingHappenedCommand(m_leds)
        //     )
        // );

        // TODO: implement this
        // L1 - intake
        // R1 - outtake
        // Right d-pad - climb
        // Left d-pad - abort climb
    }

    public Command getAutonomousCommand() {
        return Commands.none();
        // return AutoChooser.get().beforeStarting(
        //     SwerveCommandFactory.setPosition(m_swerve, () -> new Pose2d(8, 8, Rotation2d.k180deg))
        // );
    }

    public Command getTeleopCommand() {
        // return new AutomatedTeleopControllerListenerCommand(
        //     m_elevator,
        //     m_swerve,
        //     m_endEffector,
        //     m_leds,
        //     m_driverController,
        //     m_operatorController
        // );
        return Commands.none();
    }
}
