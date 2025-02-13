package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldConstants.CoralStationConstants.CoralStation;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;
import frc.robot.util.FieldConstants.ReefConstants.ReefHeight;

public class AutoBuilder {
    public static Command none() {
        return Commands.none();
    }

    public static Command testAuto(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffectorSubsystem
    ) {
        //        return Commands.sequence(
        //            SwerveCommandFactory.setPosition(swerve, () -> new Pose2d(8.5, 7.6, Rotation2d.fromDegrees(180))),
        //            Commands.waitSeconds(4),
        //            makeScoreCommand(ReefBranch.kL4G1, swerve, elevator, endEffectorSubsystem),
        //            makeIntakeCommand(CoralStation.kF3, swerve, elevator, endEffectorSubsystem),
        //            makeScoreCommand(ReefBranch.kL3F2, swerve, elevator, endEffectorSubsystem),
        //            makeIntakeCommand(CoralStation.kF1, swerve, elevator, endEffectorSubsystem),
        //            makeScoreCommand(ReefBranch.kL2E1, swerve, elevator, endEffectorSubsystem),
        //            makeIntakeCommand(CoralStation.kC2, swerve, elevator, endEffectorSubsystem),
        //            makeScoreCommand(ReefBranch.kL3I2, swerve, elevator, endEffectorSubsystem)
        //        ).withName("AutoBuilder::testAuto");

        return Commands.sequence(
            makeAutoDecideScoreCommand(ReefHeight.L4, swerve, elevator, endEffectorSubsystem),
            makeIntakeCommand(CoralStation.kC3, swerve, elevator, endEffectorSubsystem),
            makeAutoDecideScoreCommand(ReefHeight.L4, swerve, elevator, endEffectorSubsystem),
            makeIntakeCommand(CoralStation.kC3, swerve, elevator, endEffectorSubsystem),
            makeAutoDecideScoreCommand(ReefHeight.L4, swerve, elevator, endEffectorSubsystem),
            makeIntakeCommand(CoralStation.kC3, swerve, elevator, endEffectorSubsystem),
            makeAutoDecideScoreCommand(ReefHeight.L4, swerve, elevator, endEffectorSubsystem),
            makeIntakeCommand(CoralStation.kC3, swerve, elevator, endEffectorSubsystem),
            makeAutoDecideScoreCommand(ReefHeight.L4, swerve, elevator, endEffectorSubsystem),
            makeIntakeCommand(CoralStation.kC3, swerve, elevator, endEffectorSubsystem),
            makeAutoDecideScoreCommand(ReefHeight.L4, swerve, elevator, endEffectorSubsystem)
        );
    }

    public static Command simpleTaxiForward(SwerveSubsystem swerve) {
        return Commands.sequence(
            SwerveCommandFactory.drive(swerve, 0.0, 1.0, 0.0, false),
            Commands.waitSeconds(1),
            SwerveCommandFactory.stopAndX(swerve)
        ).withName("AutoBuilder::simpleTaxiForward");
    }

    public static Command makeScoreCommand(
        ReefBranch branch,
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector
    ) {
        return Commands.parallel(
            SwerveCommandFactory.driveToReefScore(swerve, branch),
            SuperstructureCommandFactory.autoScore(elevator, endEffector, branch)
        ).finallyDo(() -> RobotState.addScoredBranch(branch)).withName(branch + " Score Command");
    }

    public static Command makeIntakeCommand(
        CoralStation station,
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector
    ) {
        return Commands.parallel(
            SwerveCommandFactory.driveToPoseThenFollowPath(
                Constants.SwerveConstants.kPPPathFindConstraints,
                station.getInnerPath()
            ),
            ElevatorCommandFactory.toHome(elevator),
            EndEffectorCommandFactory.intake(endEffector)
        );
    }

    public static Command makeAutoDecideScoreCommand(
        ReefHeight height,
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector
    ) {
        return new AutoDecideScoreCommand(height, swerve, elevator, endEffector);
    }
}
