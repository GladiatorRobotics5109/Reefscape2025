package frc.robot.commands;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.superstructure.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Conversions;
import frc.robot.util.FieldConstants.CoralStationConstants.CoralStation;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;
import frc.robot.util.FieldConstants.ReefConstants.ReefHeight;
import frc.robot.util.Paths;
import frc.robot.util.Util;

import java.util.List;
import java.util.function.Supplier;

public class AutoBuilder {
    public static Command none(SwerveSubsystem swerve) {
        return prefix(swerve);
    }

    public static Command testAuto(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef = Paths.ppPaths.get("TestPath");
        final ReefBranch kBranch = ReefBranch.kL4F2;

        return Commands.sequence(
            prefix(swerve, kToReef),
            SwerveCommandFactory.followPath(swerve, kToReef),
            score(kBranch, swerve, elevator, endEffector, leds)
        );
    }

    public static Command followTestPath(SwerveSubsystem swerve) {
        PathPlannerPath path = Paths.ppPaths.get("TestPath");
        return Commands.sequence(
            prefix(swerve, path),
            SwerveCommandFactory.followPath(swerve, path)
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

    // public static Command simpleL2(
    //     SwerveSubsystem swerve,
    //     ElevatorSubsystem elevator,
    //     EndEffectorSubsystem endEffector,
    //     LEDSubsystem leds
    // ) {
    //     return simpleReefHeight(
    //         0.4,
    //         Conversions.inchesToMeters(87.947),
    //         ReefHeight.L2,
    //         swerve,
    //         elevator,
    //         endEffector,
    //         leds
    //     );
    // }

    public static Command simpleL1(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        final double kDriveSpeed = 0.4;
        final double kDriveDistance = Conversions.inchesToMeters(87.947);

        return Commands.sequence(
            prefix(
                swerve,
                () -> new Pose2d(0.0, 0.0, Util.getAlliance() == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg)
            ),
            SwerveCommandFactory.drive(swerve, kDriveSpeed, 0.0, 0.0, false),
            Commands.waitSeconds((1 / kDriveSpeed) * kDriveDistance + 0.1),
            SwerveCommandFactory.drive(swerve, 0.0, 0.0, 0.0, false),
            ElevatorCommandFactory.toReefHeight(elevator, ReefHeight.L1),
            // ElevatorCommandFactory.waitSetpoint(elevator),
            Commands.waitSeconds(3.5),
            EndEffectorCommandFactory.scoreL1WithTimeout(endEffector),
            LEDCommandFactory.goodThingHappenedCommand(leds),
            SwerveCommandFactory.drive(swerve, -kDriveSpeed, 0.0, 0.0, false),
            Commands.waitSeconds(0.6),
            SwerveCommandFactory.drive(swerve, 0.0, 0.0, 0.0, false)
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

    public static Command lessSimpleL4(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        final double kDriveSpeed = 0.4;

        return Commands.sequence(
            prefix(
                swerve,
                () -> {
                    //                    Translation2d branchPosition = flipIfNecessary(
                    //                        ReefBranch.kL4H1.getBranchPosition().toTranslation2d()
                    //                    );
                    //
                    //                    Logger.recordOutput("TestPose5", new Translation3d(branchPosition));
                    //                    Translation2d startingPosition = branchPosition.plus(
                    //                        new Translation2d(
                    //                            Conversions.inchesToMeters(88.0) - Math.abs(SwerveModuleConstants.kModulePosFL.getX()),
                    //                            Util.getAlliance() == Alliance.Blue ? Rotation2d.kZero : Rotation2d.kPi
                    //                        )
                    //                    );
                    //                    Logger.recordOutput("TestPose6", new Translation3d(startingPosition));
                    //
                    //                    return new Pose2d(
                    //                        startingPosition,
                    //                        Util.getAlliance() == Alliance.Blue ? Rotation2d.kPi : Rotation2d.kZero
                    //                    );
                    return Util.getAlliance() == Alliance.Blue
                        ? new Pose2d(7.20, 4.187, Rotation2d.kPi)
                        : new Pose2d(10.345, 4.187, Rotation2d.kZero);
                }
            ),
            SwerveCommandFactory.drive(swerve, kDriveSpeed, 0.0, 0.0, false),
            Commands.waitSeconds(1.0),
            SwerveCommandFactory.drive(swerve, 0.0, 0.0, 0.0, false),
            score(ReefBranch.kL4H1, swerve, elevator, endEffector, leds)
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
            prefix(
                swerve,
                () -> new Pose2d(0.0, 0.0, Util.getAlliance() == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg)
            ),
            SwerveCommandFactory.drive(swerve, driveSpeedMetersPerSecond, 0.0, 0.0, false),
            Commands.waitSeconds((1 / driveSpeedMetersPerSecond) * driveDistanceMeters + 0.1),
            SwerveCommandFactory.drive(swerve, 0.0, 0.0, 0.0, false),
            ElevatorCommandFactory.toReefHeight(elevator, height),
            // ElevatorCommandFactory.waitSetpoint(elevator),
            Commands.waitSeconds(3.5),
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
        IntakeSubsystem intake,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        return Commands.parallel(
            SwerveCommandFactory.followPath(swerve, path),
            SuperstructureCommandFactory.intake(elevator, intake, endEffector)
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

    public static Command auto_PP_Right_1L4(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef = Paths.ppPaths.get("B_2-R_I1");
        final ReefBranch kBranch = ReefBranch.kL4I1;

        return Commands.sequence(
            prefix(swerve, kToReef),
            SwerveCommandFactory.followPath(swerve, kToReef),
            score(kBranch, swerve, elevator, endEffector, leds)
        );
    }

    public static Command auto_PP_Right_2L4(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        IntakeSubsystem intake,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef1 = Paths.ppPaths.get("B_2-R_I1");
        final ReefBranch kBranch1 = ReefBranch.kL4I1;
        final PathPlannerPath kToCoral1 = Paths.ppPaths.get("R_I1-C_C3");
        final ReefBranch kBranch2 = ReefBranch.kL4J2;
        final PathPlannerPath kToReef2 = Paths.ppPaths.get("C_C3-R_J2");

        return Commands.sequence(
            prefix(swerve, kToReef1),
            SwerveCommandFactory.followPath(swerve, kToReef1),
            score(kBranch1, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral1, swerve, elevator, intake, endEffector, leds),
            SwerveCommandFactory.followPath(swerve, kToReef2),
            score(kBranch2, swerve, elevator, endEffector, leds)
        );
    }

    public static Command auto_PP_Right_3L4(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        IntakeSubsystem intake,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef1 = Paths.ppPaths.get("B_2-R_I1");
        final ReefBranch kBranch1 = ReefBranch.kL4I1;
        final PathPlannerPath kToCoral1 = Paths.ppPaths.get("R_I1-C_C3");
        final PathPlannerPath kToReef2 = Paths.ppPaths.get("C_C3-R_J2");
        final ReefBranch kBranch2 = ReefBranch.kL4J2;
        final PathPlannerPath kToCoral2 = Paths.ppPaths.get("R_J2-C_C3");
        final PathPlannerPath kToReef3 = Paths.ppPaths.get("C_C3-R_J1");
        final ReefBranch kBranch3 = ReefBranch.kL4J1;

        return Commands.sequence(
            prefix(swerve, kToReef1),
            SwerveCommandFactory.followPath(swerve, kToReef1),
            score(kBranch1, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral1, swerve, elevator, intake, endEffector, leds),
            SwerveCommandFactory.followPath(swerve, kToReef2),
            score(kBranch2, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral2, swerve, elevator, intake, endEffector, leds),
            SwerveCommandFactory.followPath(swerve, kToReef3),
            score(kBranch3, swerve, elevator, endEffector, leds)
        );
    }

    public static Command auto_PP_Right_4L4(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        IntakeSubsystem intake,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef1 = Paths.ppPaths.get("B_2-R_I1");
        final ReefBranch kBranch1 = ReefBranch.kL4I1;
        final PathPlannerPath kToCoral1 = Paths.ppPaths.get("R_I1-C_C3");
        final PathPlannerPath kToReef2 = Paths.ppPaths.get("C_C3-R_J2");
        final ReefBranch kBranch2 = ReefBranch.kL4J2;
        final PathPlannerPath kToCoral2 = Paths.ppPaths.get("R_J2-C_C3");
        final PathPlannerPath kToReef3 = Paths.ppPaths.get("C_C3-R_J1");
        final ReefBranch kBranch3 = ReefBranch.kL4J1;
        final PathPlannerPath kToCoral3 = Paths.ppPaths.get("R_J1-C_C3");
        final PathPlannerPath kToReef4 = Paths.ppPaths.get("C_C3-R_E2");
        final ReefBranch kBranch4 = ReefBranch.kL4E2;

        return Commands.sequence(
            prefix(swerve, kToReef1),
            SwerveCommandFactory.followPath(swerve, kToReef1),
            score(kBranch1, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral1, swerve, elevator, intake, endEffector, leds),
            SwerveCommandFactory.followPath(swerve, kToReef2),
            score(kBranch2, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral2, swerve, elevator, intake, endEffector, leds),
            SwerveCommandFactory.followPath(swerve, kToReef3),
            score(kBranch3, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral3, swerve, elevator, intake, endEffector, leds),
            SwerveCommandFactory.followPath(swerve, kToReef4),
            score(kBranch4, swerve, elevator, endEffector, leds)
        );
    }

    public static Command auto_PP_Left_1L4(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef = flipOverX(Paths.ppPaths.get("B_2-R_I1"));
        final ReefBranch kBranch = ReefBranch.kL4G1;

        return Commands.sequence(
            prefix(swerve, kToReef),
            SwerveCommandFactory.followPath(swerve, kToReef),
            score(kBranch, swerve, elevator, endEffector, leds)
        );
    }

    public static Command auto_PP_Left_2L4(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        IntakeSubsystem intake,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef1 = flipOverX(Paths.ppPaths.get("B_2-R_I1"));
        final ReefBranch kBranch1 = ReefBranch.kL4G1;
        final PathPlannerPath kToCoral1 = flipOverX(Paths.ppPaths.get("R_I1-C_C3"));
        final ReefBranch kBranch2 = ReefBranch.kL4F2;
        final PathPlannerPath kToReef2 = flipOverX(Paths.ppPaths.get("C_C3-R_J2"));

        return Commands.sequence(
            prefix(swerve, kToReef1),
            SwerveCommandFactory.followPath(swerve, kToReef1),
            score(kBranch1, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral1, swerve, elevator, intake, endEffector, leds),
            SwerveCommandFactory.followPath(swerve, kToReef2),
            score(kBranch2, swerve, elevator, endEffector, leds)
        );
    }

    public static Command auto_PP_Left_3L4(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        IntakeSubsystem intake,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef1 = flipOverX(Paths.ppPaths.get("B_2-R_I1"));
        final ReefBranch kBranch1 = ReefBranch.kL4G1;
        final PathPlannerPath kToCoral1 = flipOverX(Paths.ppPaths.get("R_I1-C_C3"));
        final PathPlannerPath kToReef2 = flipOverX(Paths.ppPaths.get("C_C3-R_J2"));
        final ReefBranch kBranch2 = ReefBranch.kL4F2;
        final PathPlannerPath kToCoral2 = flipOverX(Paths.ppPaths.get("R_J2-C_C3"));
        final PathPlannerPath kToReef3 = flipOverX(Paths.ppPaths.get("C_C3-R_J1"));
        final ReefBranch kBranch3 = ReefBranch.kL4F1;

        return Commands.sequence(
            prefix(swerve, kToReef1),
            SwerveCommandFactory.followPath(swerve, kToReef1),
            score(kBranch1, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral1, swerve, elevator, intake, endEffector, leds),
            SwerveCommandFactory.followPath(swerve, kToReef2),
            score(kBranch2, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral2, swerve, elevator, intake, endEffector, leds),
            SwerveCommandFactory.followPath(swerve, kToReef3),
            score(kBranch3, swerve, elevator, endEffector, leds)
        );
    }

    public static Command auto_PP_Left_4L4(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        IntakeSubsystem intake,
        LEDSubsystem leds
    ) {
        final PathPlannerPath kToReef1 = flipOverX(Paths.ppPaths.get("B_2-R_I1"));
        final ReefBranch kBranch1 = ReefBranch.kL4G1;
        final PathPlannerPath kToCoral1 = flipOverX(Paths.ppPaths.get("R_I1-C_C3"));
        final PathPlannerPath kToReef2 = flipOverX(Paths.ppPaths.get("C_C3-R_J2"));
        final ReefBranch kBranch2 = ReefBranch.kL4F2;
        final PathPlannerPath kToCoral2 = flipOverX(Paths.ppPaths.get("R_J2-C_C3"));
        final PathPlannerPath kToReef3 = flipOverX(Paths.ppPaths.get("C_C3-R_J1"));
        final ReefBranch kBranch3 = ReefBranch.kL4F1;
        final PathPlannerPath kToCoral3 = flipOverX(Paths.ppPaths.get("R_J1-C_C3"));
        final PathPlannerPath kToReef4 = flipOverX(Paths.ppPaths.get("C_C3-R_E2"));
        final ReefBranch kBranch4 = ReefBranch.kL4E1;

        return Commands.sequence(
            prefix(swerve, kToReef1),
            SwerveCommandFactory.followPath(swerve, kToReef1),
            score(kBranch1, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral1, swerve, elevator, intake, endEffector, leds),
            SwerveCommandFactory.followPath(swerve, kToReef2),
            score(kBranch2, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral2, swerve, elevator, intake, endEffector, leds),
            SwerveCommandFactory.followPath(swerve, kToReef3),
            score(kBranch3, swerve, elevator, endEffector, leds),
            followCoralPathAndIntake(kToCoral3, swerve, elevator, intake, endEffector, leds),
            SwerveCommandFactory.followPath(swerve, kToReef4),
            score(kBranch4, swerve, elevator, endEffector, leds)
        );
    }

    public static Command score(
        ReefBranch branch,
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        Pose2d targetPose = flipIfNecessary(branch.getSwerveTargetPoseInner());
        Pose2d outerPose = flipIfNecessary(branch.getSwerveTargetPoseOuter());

        ChassisSpeeds leaveSpeeds = new ChassisSpeeds(
            -SwerveConstants.kAutoScoreLeaveSpeed,
            0.0,
            0.0
        );

        // if (Util.getAlliance() == Alliance.Red) {
        //     leaveSpeeds.vxMetersPerSecond = -leaveSpeeds.vxMetersPerSecond;
        //     leaveSpeeds.vyMetersPerSecond = -leaveSpeeds.vyMetersPerSecond;
        // }

        return Commands.sequence(
            Commands.parallel(
                SwerveCommandFactory.driveToPose(swerve, targetPose),
                ElevatorCommandFactory.toReefBranch(elevator, branch)
            ),
            EndEffectorCommandFactory.score(endEffector, branch),
            ElevatorCommandFactory.toHome(elevator),
            LEDCommandFactory.goodThingHappenedCommand(leds),
            Commands.waitUntil(elevator::isSafeToAccelerate),
            SwerveCommandFactory.drive(swerve, leaveSpeeds, false),
            Commands.waitSeconds(0.4 / SwerveConstants.kAutoScoreLeaveSpeed + 0.1),
            SwerveCommandFactory.drive(swerve, 0.0, 0.0, 0.0, true)
        ).withName("Score " + branch);
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

    public static Command pushTaxi(SwerveSubsystem swerve) {
        return Commands.sequence(
            SwerveCommandFactory.drive(swerve, -1.0, 0.0, 0.0, false),
            Commands.waitSeconds(0.5),
            SwerveCommandFactory.drive(swerve, 0.0, 0.0, 0.0, false)
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

    public static PathPlannerPath flipOverX(PathPlannerPath path) {
        List<Waypoint> waypoints = path.getWaypoints().stream().map(AutoBuilder::flipOverX).toList();
        List<RotationTarget> rotationTargets = path.getRotationTargets().stream().map(AutoBuilder::flipOverX).toList();
        List<PointTowardsZone> pointTowardsZones = path.getPointTowardsZones().stream().map(AutoBuilder::flipOverX)
            .toList();
        List<ConstraintsZone> constraintZones = path.getConstraintZones();
        List<EventMarker> eventMarkers = path.getEventMarkers();
        PathConstraints globalConstraints = path.getGlobalConstraints();
        IdealStartingState idealStartingState = path.getIdealStartingState() != null
            ? flipOverX(path.getIdealStartingState())
            : null;
        GoalEndState goalEndState = flipOverX(path.getGoalEndState());
        //        List<PathPoint> allPoints = path.getAllPathPoints().stream().map(AutoBuilder::flipOverX).toList();
        boolean reversed = path.isReversed();
        //        boolean isChoreoPath = path.isChoreoPath();
        // PathPlannerTrajectory idealTrajectory =
        //        boolean preventFlipping = path.preventFlipping;
        //        String name = path.name;

        return new PathPlannerPath(
            waypoints,
            rotationTargets,
            pointTowardsZones,
            constraintZones,
            eventMarkers,
            globalConstraints,
            idealStartingState,
            goalEndState,
            reversed
        );
    }

    public static Waypoint flipOverX(Waypoint waypoint) {
        Translation2d prevControl = waypoint.prevControl();
        Translation2d anchor = waypoint.anchor();
        Translation2d nextControl = waypoint.nextControl();

        return new Waypoint(
            prevControl != null ? flipOverX(prevControl) : null,
            anchor != null ? flipOverX(anchor) : null,
            nextControl != null ? flipOverX(nextControl) : null
        );
    }

    public static Translation2d flipOverX(Translation2d pos) {
        return new Translation2d(
            pos.getX(),
            (pos.getY() - FlippingUtil.fieldSizeY / 2.0) * -1 + FlippingUtil.fieldSizeY / 2.0
        );
    }

    public static Rotation2d flipOverX(Rotation2d rotation) {
        return rotation.times(-1.0);
    }

    public static RotationTarget flipOverX(RotationTarget rotation) {
        return new RotationTarget(rotation.position(), flipOverX(rotation.rotation()));
    }

    public static PointTowardsZone flipOverX(PointTowardsZone pointTowardsZone) {
        return new PointTowardsZone(
            pointTowardsZone.name(),
            flipOverX(pointTowardsZone.targetPosition()),
            pointTowardsZone.rotationOffset(),
            pointTowardsZone.minPosition(),
            pointTowardsZone.maxPosition()
        );
    }

    public static IdealStartingState flipOverX(IdealStartingState idealStartingState) {
        return new IdealStartingState(
            idealStartingState.velocityMPS(),
            flipOverX(idealStartingState.rotation())
        );
    }

    public static GoalEndState flipOverX(GoalEndState goalEndState) {
        return new GoalEndState(
            goalEndState.velocityMPS(),
            flipOverX(goalEndState.rotation())
        );
    }

    public static PathPoint flipOverX(PathPoint point) {
        return new PathPoint(flipOverX(point.position), flipOverX(point.rotationTarget), point.constraints);
    }

    //    public static Command auto_PP_B6_L2G2(
    //        SwerveSubsystem swerve,
    //        ElevatorSubsystem elevator,
    //        EndEffectorSubsystem endEffector,
    //        LEDSubsystem leds
    //    ) {
    //        final PathPlannerPath kToReef = Paths.ppPaths.get("B_6-R_G2");
    //        final ReefBranch kBranch = ReefBranch.kL2G2;
    //
    //        return Commands.sequence(
    //            prefix(swerve, kToReef),
    //            followReefPathAndScore(kToReef, kBranch, swerve, elevator, endEffector, leds)
    //        );
    //    }
    //
    //    public static Command auto_PP_B6_L2G2_F3_L2G1(
    //        SwerveSubsystem swerve,
    //        ElevatorSubsystem elevator,
    //        IntakeSubsystem intake,
    //        EndEffectorSubsystem endEffector,
    //        LEDSubsystem leds
    //    ) {
    //        final PathPlannerPath kToReef1 = Paths.ppPaths.get("B_6-R_G2");
    //        final PathPlannerPath kToCoral = Paths.ppPaths.get("R_G2-C_F3");
    //        final PathPlannerPath kToReef2 = Paths.ppPaths.get("C_F3-R_G1");
    //        final ReefBranch kBranch1 = ReefBranch.kL2G2;
    //        final ReefBranch kBranch2 = ReefBranch.kL2G1;
    //
    //        return Commands.sequence(
    //            prefix(swerve, kToReef1),
    //            followReefPathAndScore(kToReef1, kBranch1, swerve, elevator, endEffector, leds),
    //            followCoralPathAndIntake(kToCoral, swerve, elevator, intake, endEffector, leds),
    //            followReefPathAndScore(kToReef2, kBranch2, swerve, elevator, endEffector, leds)
    //        );
    //    }
    //
    //    public static Command auto_PP_B6_L4G2_Leave(
    //        SwerveSubsystem swerve,
    //        ElevatorSubsystem elevator,
    //        EndEffectorSubsystem endEffector,
    //        LEDSubsystem leds
    //    ) {
    //        final PathPlannerPath kToReef = Paths.ppPaths.get("B_6-R_G2");
    //        final PathPlannerPath kLeave = Paths.ppPaths.get("R_G2-Leave");
    //        final ReefBranch kBranch = ReefBranch.kL4G2;
    //
    //        return Commands.sequence(
    //            prefix(swerve, kToReef),
    //            followReefPathAndScore(kToReef, kBranch, swerve, elevator, endEffector, leds),
    //            Commands.parallel(
    //                SwerveCommandFactory.followPath(swerve, kLeave),
    //                ElevatorCommandFactory.toHome(elevator)
    //            )
    //        );
    //    }
    //
    //    public static Command auto_PP_B6_L4G2_F3_L4G1_Leave(
    //        SwerveSubsystem swerve,
    //        ElevatorSubsystem elevator,
    //        IntakeSubsystem intake,
    //        EndEffectorSubsystem endEffector,
    //        LEDSubsystem leds
    //    ) {
    //        final PathPlannerPath kToReef1 = Paths.ppPaths.get("B_6-R_G2");
    //        final PathPlannerPath kToCoral = Paths.ppPaths.get("R_G2-C_F3");
    //        final PathPlannerPath kToReef2 = Paths.ppPaths.get("C_F3-R_G1");
    //        final PathPlannerPath kLeave = Paths.ppPaths.get("R_G1-Leave");
    //        final ReefBranch kBranch1 = ReefBranch.kL4G2;
    //        final ReefBranch kBranch2 = ReefBranch.kL4G1;
    //
    //        return Commands.sequence(
    //            prefix(swerve, kToReef1),
    //            followReefPathAndScore(kToReef1, kBranch1, swerve, elevator, endEffector, leds),
    //            followCoralPathAndIntake(kToCoral, swerve, elevator, intake, endEffector, leds),
    //            followReefPathAndScore(kToReef2, kBranch2, swerve, elevator, endEffector, leds),
    //            Commands.parallel(
    //                SwerveCommandFactory.followPath(swerve, kLeave),
    //                ElevatorCommandFactory.toHome(elevator)
    //            )
    //        );
    //    }
    //
    //    public static Command auto_PP_BC_L2H1(
    //        SwerveSubsystem swerve,
    //        ElevatorSubsystem elevator,
    //        EndEffectorSubsystem endEffector,
    //        LEDSubsystem leds
    //    ) {
    //        final PathPlannerPath kToReef = Paths.ppPaths.get("B_C-R_H1");
    //        final ReefBranch kBranch = ReefBranch.kL2H1;
    //
    //        return Commands.sequence(
    //            prefix(swerve, kToReef),
    //            SwerveCommandFactory.followPath(swerve, kToReef),
    //            SuperstructureCommandFactory.autoScore(elevator, endEffector, leds, kBranch),
    //            ElevatorCommandFactory.toHome(elevator)
    //        );
    //    }
    //
    //    public static Command auto_PP_B6_3L2(
    //        SwerveSubsystem swerve,
    //        ElevatorSubsystem elevator,
    //        IntakeSubsystem intake,
    //        EndEffectorSubsystem endEffector,
    //        LEDSubsystem leds
    //    ) {
    //        final PathPlannerPath kToReef1 = Paths.ppPaths.get("B_6-R_G2");
    //        final PathPlannerPath kToCoral1 = Paths.ppPaths.get("R_G2-C_F3");
    //        final PathPlannerPath kToReef2 = Paths.ppPaths.get("C_F3-R_G1");
    //        final PathPlannerPath kToCoral2 = Paths.ppPaths.get("R_G1-C_F3");
    //        final PathPlannerPath kToReef3 = Paths.ppPaths.get("C_F3-R_F2");
    //        final ReefBranch kBranch1 = ReefBranch.kL2G2;
    //        final ReefBranch kBranch2 = ReefBranch.kL2G1;
    //        final ReefBranch kBranch3 = ReefBranch.kL2F1;
    //
    //        return Commands.sequence(
    //            prefix(swerve, kToReef1),
    //            followReefPathAndScore(kToReef1, kBranch1, swerve, elevator, endEffector, leds),
    //            followCoralPathAndIntake(kToCoral1, swerve, elevator, intake, endEffector, leds),
    //            followReefPathAndScore(kToReef2, kBranch2, swerve, elevator, endEffector, leds),
    //            followCoralPathAndIntake(kToCoral2, swerve, elevator, intake, endEffector, leds),
    //            followReefPathAndScore(kToReef3, kBranch3, swerve, elevator, endEffector, leds),
    //            ElevatorCommandFactory.toHome(elevator)
    //        );
    //    }
    //
    //    public static Command auto_PP_B6_3L4(
    //        SwerveSubsystem swerve,
    //        ElevatorSubsystem elevator,
    //        IntakeSubsystem intake,
    //        EndEffectorSubsystem endEffector,
    //        LEDSubsystem leds
    //    ) {
    //        final PathPlannerPath kToReef1 = Paths.ppPaths.get("B_6-R_G2");
    //        final PathPlannerPath kToCoral1 = Paths.ppPaths.get("R_G2-C_F3");
    //        final PathPlannerPath kToReef2 = Paths.ppPaths.get("C_F3-R_G1");
    //        final PathPlannerPath kToCoral2 = Paths.ppPaths.get("R_G1-C_F3");
    //        final PathPlannerPath kToReef3 = Paths.ppPaths.get("C_F3-R_F2");
    //        final ReefBranch kBranch1 = ReefBranch.kL4G2;
    //        final ReefBranch kBranch2 = ReefBranch.kL4G1;
    //        final ReefBranch kBranch3 = ReefBranch.kL4F1;
    //
    //        return Commands.sequence(
    //            prefix(swerve, kToReef1),
    //            followReefPathAndScore(kToReef1, kBranch1, swerve, elevator, endEffector, leds),
    //            followCoralPathAndIntake(kToCoral1, swerve, elevator, intake, endEffector, leds),
    //            followReefPathAndScore(kToReef2, kBranch2, swerve, elevator, endEffector, leds),
    //            followCoralPathAndIntake(kToCoral2, swerve, elevator, intake, endEffector, leds),
    //            followReefPathAndScore(kToReef3, kBranch3, swerve, elevator, endEffector, leds),
    //            ElevatorCommandFactory.toHome(elevator)
    //        );
    //    }
}
