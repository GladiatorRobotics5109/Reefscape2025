package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Conversions;
import frc.robot.util.FieldConstants.CoralStationConstants.CoralStation;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;
import frc.robot.util.FieldConstants.ReefConstants.ReefHeight;
import frc.robot.util.Paths;
import frc.robot.util.Util;

import java.util.function.Supplier;

public class AutoBuilder {
    public static Command none(SwerveSubsystem swerve) {
        return prefix(swerve);
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

    public static Command followTestPath(SwerveSubsystem swerve) {
        return Commands.sequence(
            SwerveCommandFactory.setPosition(swerve, () -> new Pose2d(6.0, 3.5, Rotation2d.k180deg)),
            SwerveCommandFactory.followPath(swerve, Paths.ppPaths.get("testPath"))
        );
    }

    public static Command simpleTaxiForward(SwerveSubsystem swerve) {
        return Commands.sequence(
            prefix(swerve),
            SwerveCommandFactory.drive(swerve, 1.0, 0.0, 0.0, false),
            Commands.waitSeconds(2),
            SwerveCommandFactory.stopAndX(swerve)
        ).withName("AutoBuilder::simpleTaxiForward");
    }

    public static Command simpleL2(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        return simpleReefHeight(
            0.4,
            Conversions.inchesToMeters(87.947),
            ReefHeight.L2,
            swerve,
            elevator,
            endEffector,
            leds
        );
    }

    public static Command simpleL4(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        return simpleReefHeight(
            0.4,
            Conversions.inchesToMeters(87.947),
            ReefHeight.L4,
            swerve,
            elevator,
            endEffector,
            leds
        );
    }

    public static Command simpleReefHeight(
        double driveSpeedMetersPerSecond,
        double driveDistanceMeters,
        ReefHeight height,
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        return Commands.sequence(
            SwerveCommandFactory.drive(swerve, driveSpeedMetersPerSecond, 0.0, 0.0, false),
            Commands.waitSeconds((1 / driveSpeedMetersPerSecond) * driveDistanceMeters),
            SwerveCommandFactory.drive(swerve, 0.0, 0.0, 0.0, false),
            ElevatorCommandFactory.toReefHeight(elevator, height),
            ElevatorCommandFactory.waitSetpoint(elevator),
            Commands.waitSeconds(1.0),
            EndEffectorCommandFactory.scoreWithTimeout(endEffector),
            LEDCommandFactory.goodThingHappenedCommand(leds)
        );
    }

    public static Command followPathToReef(SwerveSubsystem swerve, PathPlannerPath path, ReefBranch branch) {
        return Commands.sequence(
            SwerveCommandFactory.followPath(swerve, path),
            SwerveCommandFactory.driveToPose(swerve, branch.getSwerveTargetPoseInner())
        );
    }

    public static Command followReefPathAndScore(
        PathPlannerPath path,
        ReefBranch branch,
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {

        return Commands.parallel(
            followPathToReef(swerve, path, branch),
            SuperstructureCommandFactory.autoScore(elevator, endEffector, leds, branch).andThen(
                LEDCommandFactory.goodThingHappenedCommand(leds)
            )
        );
    }

    public static Command followCoralPathAndIntake(
        PathPlannerPath path,
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        return Commands.parallel(
            SwerveCommandFactory.followPath(swerve, path),
            SuperstructureCommandFactory.intake(elevator, endEffector)
        );
    }

    //    public static Command auto_PP_B6_L1G2(
    //        SwerveSubsystem swerve,
    //        ElevatorSubsystem elevator,
    //        EndEffectorSubsystem endEffector,
    //        LEDSubsystem leds
    //    ) {
    //        final PathPlannerPath kToReef = Paths.ppPaths.get("B_6-R_G2");
    //
    //        return Commands.sequence(
    //            prefix(swerve, kToReef),
    //            followReefPathAndScore(kToReef, ReefBranch.kG2k, swerve, elevator, endEffector, leds)
    //        );
    //    }

    public static Command auto_PP_B6_L2G2(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef = Paths.ppPaths.get("B_6-R_G2");
        final ReefBranch kBranch = ReefBranch.kL2G2;

        return Commands.sequence(
            prefix(swerve, kToReef),
            followReefPathAndScore(kToReef, kBranch, swerve, elevator, endEffector, leds)
        );
    }

    public static Command auto_PP_B6_L2G2_F3_L2G1(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef1 = Paths.ppPaths.get("B_6-R_G2");
        final PathPlannerPath kToCoral = Paths.ppPaths.get("R_G2-C_F3");
        final PathPlannerPath kToReef2 = Paths.ppPaths.get("C_F3-R_G1");
        final ReefBranch kBranch1 = ReefBranch.kL2G2;
        final ReefBranch kBranch2 = ReefBranch.kL2G1;

        return Commands.sequence(
            prefix(swerve, kToReef1),
            followReefPathAndScore(kToReef1, kBranch1, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral, swerve, elevator, endEffector, leds),
            followReefPathAndScore(kToReef2, kBranch2, swerve, elevator, endEffector, leds)
        );
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
            prefix(swerve, kToReef),
            followReefPathAndScore(kToReef, kBranch, swerve, elevator, endEffector, leds),
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
            prefix(swerve, kToReef1),
            followReefPathAndScore(kToReef1, kBranch1, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral, swerve, elevator, endEffector, leds),
            followReefPathAndScore(kToReef2, kBranch2, swerve, elevator, endEffector, leds),
            Commands.parallel(
                SwerveCommandFactory.followPath(swerve, kLeave),
                ElevatorCommandFactory.toHome(elevator)
            )
        );
    }

    public static Command auto_PP_BC_L2H1(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef = Paths.ppPaths.get("B_C-R_H1");
        final ReefBranch kBranch = ReefBranch.kL2H1;

        return Commands.sequence(
            prefix(swerve, kToReef),
            SwerveCommandFactory.followPath(swerve, kToReef),
            SuperstructureCommandFactory.autoScore(elevator, endEffector, leds, kBranch),
            ElevatorCommandFactory.toHome(elevator)
        );
    }

    public static Command auto_PP_B6_3L2(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef1 = Paths.ppPaths.get("B_6-R_G2");
        final PathPlannerPath kToCoral1 = Paths.ppPaths.get("R_G2-C_F3");
        final PathPlannerPath kToReef2 = Paths.ppPaths.get("C_F3-R_G1");
        final PathPlannerPath kToCoral2 = Paths.ppPaths.get("R_G1-C_F3");
        final PathPlannerPath kToReef3 = Paths.ppPaths.get("C_F3-R_F2");
        final ReefBranch kBranch1 = ReefBranch.kL2G2;
        final ReefBranch kBranch2 = ReefBranch.kL2G1;
        final ReefBranch kBranch3 = ReefBranch.kL2F1;

        return Commands.sequence(
            prefix(swerve, kToReef1),
            followReefPathAndScore(kToReef1, kBranch1, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral1, swerve, elevator, endEffector, leds),
            followReefPathAndScore(kToReef2, kBranch2, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral2, swerve, elevator, endEffector, leds),
            followReefPathAndScore(kToReef3, kBranch3, swerve, elevator, endEffector, leds),
            ElevatorCommandFactory.toHome(elevator)
        );
    }

    public static Command auto_PP_B6_3L4(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef1 = Paths.ppPaths.get("B_6-R_G2");
        final PathPlannerPath kToCoral1 = Paths.ppPaths.get("R_G2-C_F3");
        final PathPlannerPath kToReef2 = Paths.ppPaths.get("C_F3-R_G1");
        final PathPlannerPath kToCoral2 = Paths.ppPaths.get("R_G1-C_F3");
        final PathPlannerPath kToReef3 = Paths.ppPaths.get("C_F3-R_F2");
        final ReefBranch kBranch1 = ReefBranch.kL4G2;
        final ReefBranch kBranch2 = ReefBranch.kL4G1;
        final ReefBranch kBranch3 = ReefBranch.kL4F1;

        return Commands.sequence(
            prefix(swerve, kToReef1),
            followReefPathAndScore(kToReef1, kBranch1, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral1, swerve, elevator, endEffector, leds),
            followReefPathAndScore(kToReef2, kBranch2, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral2, swerve, elevator, endEffector, leds),
            followReefPathAndScore(kToReef3, kBranch3, swerve, elevator, endEffector, leds),
            ElevatorCommandFactory.toHome(elevator)
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

    public static Command prefix(SwerveSubsystem swerve) {
        return prefix(
            swerve,
            () -> new Pose2d(0, 0, Util.getAlliance() == Alliance.Blue ? Rotation2d.k180deg : Rotation2d.kZero)
        );
    }

    public static Command prefix(SwerveSubsystem swerve, Supplier<Pose2d> pose) {
        return SwerveCommandFactory.setPosition(swerve, pose);
    }

    public static Command prefix(SwerveSubsystem swerve, PathPlannerPath path) {
        return prefix(swerve, () -> flipIfNecessary(path).getStartingHolonomicPose().orElse(Pose2d.kZero));
    }

    public static PathPlannerPath flipIfNecessary(PathPlannerPath path) {
        return shouldFlip() ? path.flipPath() : path;
    }

    public static Pose2d flipIfNecessary(Pose2d pose) {
        return shouldFlip() ? FlippingUtil.flipFieldPose(pose) : pose;
    }

    public static Translation2d flipIfNecessary(Translation2d position) {
        return shouldFlip() ? FlippingUtil.flipFieldPosition(position) : position;
    }

    public static boolean shouldFlip() {
        return Util.getAlliance() == Alliance.Red; // Flip if red alliance
    }
}
