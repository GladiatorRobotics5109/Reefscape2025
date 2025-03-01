// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.FieldConstants.ReefConstants.ReefHeight;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private SwerveSubsystem m_swerve;
    private VisionSubsystem m_vision;
    private ElevatorSubsystem m_elevator;
    private EndEffectorSubsystem m_endEffector;
    //    private ClimbSubsystem m_climb;
    private LEDSubsystem m_leds;

    private final CommandXboxController m_driverController;

    private final CommandXboxController m_operatorController;

    public RobotContainer() {
        m_swerve = new SwerveSubsystem();
        m_vision = new VisionSubsystem();
        m_elevator = new ElevatorSubsystem();
        m_endEffector = new EndEffectorSubsystem();
        //        m_climb = new ClimbSubsystem();
        m_leds = new LEDSubsystem();
        RobotState.init(m_swerve, m_vision, m_elevator, m_endEffector);
        AutoChooser.init(m_swerve, m_elevator, m_endEffector, m_leds);

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
        m_swerve.setDefaultCommand(
            SwerveCommandFactory.makeTeleop(m_swerve, m_driverController).onlyWhile(
                () -> DriverStation.isTeleop() || DriverStation.isTest()
            )
        );

        // Elevator setpoints
        // m_driverController.cross().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L1));
        // m_driverController.circle().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L2));
        // m_driverController.triangle().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L3));
        // m_driverController.square().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L4));
        m_driverController.a().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L1));
        m_driverController.b().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L2));
        m_driverController.y().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L3));
        m_driverController.x().onTrue(ElevatorCommandFactory.toReefHeight(m_elevator, ReefHeight.L4));

        m_driverController.povUp().whileTrue(ElevatorCommandFactory.setVoltage(m_elevator, 5)).onFalse(
            ElevatorCommandFactory.setVoltage(m_elevator, 0.0)
        );
        m_driverController.povDown().whileTrue(ElevatorCommandFactory.setVoltage(m_elevator, -5)).onFalse(
            ElevatorCommandFactory.setVoltage(m_elevator, 0.0)
        );

        //manual elevator binded to right and left trigger
        m_driverController.rightTrigger().whileTrue(ElevatorCommandFactory.setVoltage(m_elevator, 12)).onFalse(
            ElevatorCommandFactory.setVoltage(m_elevator, 0.3)
        );

        m_driverController.leftTrigger().whileTrue(ElevatorCommandFactory.setVoltage(m_elevator, -12)).onFalse(
            ElevatorCommandFactory.setVoltage(m_elevator, 0.5)
        );

        m_driverController.leftBumper().onTrue(SuperstructureCommandFactory.intake(m_elevator, m_endEffector));
        m_driverController.rightBumper().onTrue(EndEffectorCommandFactory.score(m_endEffector));

        //        m_driverController.povUp().onTrue(ClimbCommandFactory.prepareClimb(m_climb));
        //        m_driverController.povDown().onTrue(ClimbCommandFactory.climb(m_climb));

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
    }

    public Command getAutonomousCommand() { return AutoChooser.get(); }

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
