package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldConstants.CoralStationConstants.CoralStation;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;

public class AutoBuilder {
    public static
        Command
        testAuto(SwerveSubsystem swerve, ElevatorSubsystem elevator, EndEffectorSubsystem endEffectorSubsystem) {
        return Commands.sequence(
            SwerveCommandFactory.setPosition(swerve, () -> new Pose2d(8.5, 7.6, Rotation2d.fromDegrees(180))),
            Commands.waitSeconds(4),
            ReefBranch.kL4G1.makeScoreCommand(swerve, elevator, endEffectorSubsystem),
            CoralStation.kF3.makeIntakeCommand(swerve, elevator, endEffectorSubsystem),
            ReefBranch.kL3F2.makeScoreCommand(swerve, elevator, endEffectorSubsystem),
            CoralStation.kF1.makeIntakeCommand(swerve, elevator, endEffectorSubsystem),
            ReefBranch.kL2E1.makeScoreCommand(swerve, elevator, endEffectorSubsystem),
            CoralStation.kC2.makeIntakeCommand(swerve, elevator, endEffectorSubsystem),
            ReefBranch.kL3I2.makeScoreCommand(swerve, elevator, endEffectorSubsystem)
        );
    }
}
