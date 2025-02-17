package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldConstants.CoralStationConstants.CoralStation;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;
import frc.robot.util.FieldConstants.ReefConstants.ReefHeight;
import frc.robot.util.Paths;

public class AutoBuilder {
    public static Command none() {
        return Commands.none();
    }

    public static Command testAuto(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffectorSubsystem,
        LEDSubsystem leds
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
            makeAutoDecideScoreCommand(ReefHeight.L4, swerve, elevator, endEffectorSubsystem, leds),
            makeIntakeCommand(CoralStation.kC3, swerve, elevator, endEffectorSubsystem),
            makeAutoDecideScoreCommand(ReefHeight.L4, swerve, elevator, endEffectorSubsystem, leds),
            makeIntakeCommand(CoralStation.kC3, swerve, elevator, endEffectorSubsystem),
            makeAutoDecideScoreCommand(ReefHeight.L4, swerve, elevator, endEffectorSubsystem, leds),
            makeIntakeCommand(CoralStation.kC3, swerve, elevator, endEffectorSubsystem),
            makeAutoDecideScoreCommand(ReefHeight.L4, swerve, elevator, endEffectorSubsystem, leds),
            makeIntakeCommand(CoralStation.kC3, swerve, elevator, endEffectorSubsystem),
            makeAutoDecideScoreCommand(ReefHeight.L4, swerve, elevator, endEffectorSubsystem, leds),
            makeIntakeCommand(CoralStation.kC3, swerve, elevator, endEffectorSubsystem),
            makeAutoDecideScoreCommand(ReefHeight.L4, swerve, elevator, endEffectorSubsystem, leds)
        );
    }

    public static Command simpleTaxiForward(SwerveSubsystem swerve) {
        return Commands.sequence(
            SwerveCommandFactory.drive(swerve, 0.0, 1.0, 0.0, false),
            Commands.waitSeconds(1),
            SwerveCommandFactory.stopAndX(swerve)
        ).withName("AutoBuilder::simpleTaxiForward");
    }

    public static Command auto_PP_B6_L4G2_Leave(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef = Paths.ppPaths.get("B_6-R_G2");
        final PathPlannerPath kLeave = Paths.ppPaths.get("R_G2-Leave");
        final ReefBranch kBranch = ReefBranch.kL4G2;

        return Commands.sequence(
            Commands.parallel(
                SwerveCommandFactory.followPath(swerve, kToReef),
                SuperstructureCommandFactory.autoScore(elevator, endEffector, leds, kBranch)
            ),
            Commands.parallel(
                SwerveCommandFactory.followPath(swerve, kLeave),
                ElevatorCommandFactory.toHome(elevator)
            )
        );
    }

    public static Command auto_PP_B6_L4G2_F3_L4G1_Leave(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef1 = Paths.ppPaths.get("B_6-R_G2");
        final PathPlannerPath kToCoral = Paths.ppPaths.get("R_G2-C_F3");
        final PathPlannerPath kToReef2 = Paths.ppPaths.get("C_F3-R_G1");
        final PathPlannerPath kLeave = Paths.ppPaths.get("R_G1-Leave");
        final ReefBranch kBranch1 = ReefBranch.kL4G2;
        final ReefBranch kBranch2 = ReefBranch.kL4G1;

        return Commands.sequence(
            Commands.parallel(
                SwerveCommandFactory.followPath(swerve, kToReef1),
                SuperstructureCommandFactory.autoScore(elevator, endEffector, leds, kBranch1)
            ),
            Commands.parallel(
                SwerveCommandFactory.followPath(swerve, kToCoral),
                SuperstructureCommandFactory.intake(elevator, endEffector)
            ),
            Commands.parallel(
                SwerveCommandFactory.followPath(swerve, kToReef2),
                SuperstructureCommandFactory.autoScore(elevator, endEffector, leds, kBranch2)
            ),
            Commands.parallel(
                SwerveCommandFactory.followPath(swerve, kLeave),
                ElevatorCommandFactory.toHome(elevator)
            )
        );
    }

    public static Command makeAutoScoreCommand(
        ReefBranch branch,
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        return Commands.parallel(
            SwerveCommandFactory.driveToReefScore(swerve, branch),
            SuperstructureCommandFactory.autoScore(elevator, endEffector, leds, branch)
        ).andThen(LEDCommandFactory.goodThingHappenedCommand(leds)).finallyDo(() -> RobotState.addScoredBranch(branch))
            .withName(branch + " Score Command");
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
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        return new AutoDecideScoreCommand(height, swerve, elevator, endEffector, leds);
    }
}
