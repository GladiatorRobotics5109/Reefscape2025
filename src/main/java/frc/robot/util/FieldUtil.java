package frc.robot.util;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.elevator.ElevatorCommandFactory;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveCommandFactory;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class FieldUtil {
    public static enum ReefBranch {
        L2E1(
            new Translation3d(),
            Paths.L2E1
        );

        private final Translation3d m_branchPosition;
        private final PathPlannerPath m_innerPath;

        private ReefBranch(Translation3d branchPosition, PathPlannerPath innerPath) {
            m_branchPosition = branchPosition;
            m_innerPath = innerPath;
        }

        public Pose2d getFieldPoseOuter() {
            return m_innerPath.getStartingHolonomicPose().orElseGet(() -> {
                DriverStation.reportError("Failed to get path starting pose!", true);
                return new Pose2d();
            });
        }

        public Pose2d getFieldPoseInner() {
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
                ElevatorCommandFactory.toPosition(elevator, this)
            );
        }

        public PathPlannerPath getInnerPath() {
            return m_innerPath;
        }
    }
}
