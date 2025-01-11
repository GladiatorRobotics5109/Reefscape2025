package frc.robot.util;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.elevator.ElevatorCommandFactory;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveCommandFactory;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class FieldConstants {
    public static enum ReefBranch {
        L2E1(
            new Translation3d(
                Util.getAlliance() == Alliance.Blue
                    ? getAllainceReefPos().getX() - kReefRadiusMeters + kReefL2L3InsetMeters
                    : getAllainceReefPos().getX() + kReefRadiusMeters - kReefL2L3InsetMeters,
                getAllainceReefPos().getY() + kReefBranchDistMeters / 2,
                kReefL2HeightMeters
            ),
            Paths.L2E1
        );

        private final Translation3d m_branchPosition;
        private final PathPlannerPath m_innerPath;

        private ReefBranch(Translation3d branchPosition, PathPlannerPath innerPath) {
            m_branchPosition = branchPosition;
            // m_innerPath = AutoBuilder.shouldFlip() ? innerPath : innerPath.flipPath();
            m_innerPath = innerPath;
            m_innerPath.flipPath();
        }

        public Pose2d getSwerveTargetPoseOuter() {
            return m_innerPath.getStartingHolonomicPose().orElseGet(() -> {
                DriverStation.reportError("Failed to get path starting pose!", true);
                return new Pose2d();
            });
        }

        public Pose2d getSwerveTargetPoseInner() {
            // Probably a better way to do this
            List<Waypoint> waypoints = m_innerPath.getWaypoints();
            return new Pose2d(waypoints.get(waypoints.size() - 1).anchor(), m_innerPath.getGoalEndState().rotation());
        }

        public Translation3d getBranchPosition() {
            return m_branchPosition;
        }

        public Command makeScoreCommand(SwerveSubsystem swerve, ElevatorSubsystem elevator) {
            return Commands.parallel(
                SwerveCommandFactory.driveToReefScore(swerve, this),
                ElevatorCommandFactory.toPosition(elevator, this).beforeStarting(
                    Commands.waitUntil(elevator::isWithinRadius).withTimeout(2)
                )
            );
        }

        public PathPlannerPath getInnerPath() {
            return m_innerPath;
        }
    }

    // TODO: check these
    public static final double kReefDiameterMeters = Conversions.inchesToMeters(65.5);
    public static final double kReefRadiusMeters = kReefDiameterMeters / 2;
    // Distance form reef base to branch at L2 and L3
    public static final double kReefL2L3InsetMeters = Conversions.inchesToMeters(1 + (5 / 8));
    public static final double kReefL4InsetMeters = Conversions.inchesToMeters(1 + (1 / 8));
    public static final double kReefL2HeightMeters = Conversions.feetToMeters(2)
        + Conversions.inchesToMeters(7 + (7 / 8));
    public static final double kReefL3HeightMeters = Conversions.feetToMeters(3)
        + Conversions.inchesToMeters(11 + (5 / 8));
    public static final double kReefL4HeightMeters = Conversions.feetToMeters(6);
    // Distance between branch centers
    public static final double kReefBranchDistMeters = Conversions.inchesToMeters(12.938);
    public static final double kReefBranchDiameterMeters = Conversions.inchesToMeters(1.6);
    public static final double kReefBranchRadiusMeters = kReefBranchDiameterMeters / 2;

    public static final Translation2d kBlueReefPos = new Translation2d(
        Conversions.inchesToMeters(176.75),
        Conversions.inchesToMeters(158.5)
    );
    public static final Translation2d kRedReefPos = new Translation2d(
        Conversions.inchesToMeters(514.13),
        Conversions.inchesToMeters(158.5)
    );

    public static Translation2d getAllainceReefPos() {
        Logger.recordOutput("TestPose", new Translation3d(kBlueReefPos.getX(), kBlueReefPos.getY(), 2));
        return Util.getAlliance() == Alliance.Red ? kRedReefPos : kBlueReefPos;
    }
}
