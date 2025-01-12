package frc.robot.util;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveCommandFactory;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class FieldConstants {
    public static enum ReefFace {
        E(0),
        F(1),
        G(2),
        H(3),
        I(4),
        J(5);

        private int m_index;

        private ReefFace(int index) {
            m_index = index;
        }

        public Rotation2d getFaceAngleFieldRelative() {
            Rotation2d offset = Util.getAlliance() == Alliance.Blue ? Rotation2d.k180deg : Rotation2d.kZero;
            Rotation2d angle = Rotation2d.fromDegrees(60 * m_index).plus(offset);
            return angle;
        }

        public Rotation2d getSwerveTargetHeading() {
            return getFaceAngleFieldRelative().plus(Rotation2d.k180deg);
        }

        public Translation2d getFacePosition() {
            return new Translation2d(kReefRadiusMeters, getFaceAngleFieldRelative()).plus(getAllainceReefPos());
        }

        public int getIndex() {
            return m_index;
        }
    }

    public static enum ReefBranch {
        L2E1(
            new Translation3d(
                Util.getAlliance() == Alliance.Blue
                    ? getAllainceReefPos().getX() - kReefRadiusMeters + kReefL2L3InsetMeters
                    : getAllainceReefPos().getX() + kReefRadiusMeters - kReefL2L3InsetMeters,
                Util.getAlliance() == Alliance.Blue
                    ? getAllainceReefPos().getY() + kReefBranchDistMeters / 2
                    : getAllainceReefPos().getY() - kReefBranchDistMeters / 2,
                kReefL2HeightMeters
            ),
            ReefFace.E,
            1
        ),
        L2E2(
            new Translation3d(
                Util.getAlliance() == Alliance.Blue
                    ? getAllainceReefPos().getX() - kReefRadiusMeters + kReefL2L3InsetMeters
                    : getAllainceReefPos().getX() + kReefRadiusMeters - kReefL2L3InsetMeters,
                Util.getAlliance() == Alliance.Blue
                    ? getAllainceReefPos().getY() - kReefBranchDistMeters / 2
                    : getAllainceReefPos().getY() + kReefBranchDistMeters / 2,
                kReefL2HeightMeters
            ),
            ReefFace.E,
            2
        );

        private final Translation3d m_branchPosition;
        private final PathPlannerPath m_innerPath;
        private final ReefFace m_face;
        private final int m_num;

        private ReefBranch(Translation3d branchPosition, ReefFace face, int num) {
            Paths.init();
            m_branchPosition = branchPosition;
            // m_innerPath = AutoBuilder.shouldFlip() ? innerPath : innerPath.flipPath();
            m_face = face;
            m_num = num;

            m_innerPath = Paths.getReefInnerPath(m_face.getIndex() * 2 + (m_num - 1));
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
            return SwerveCommandFactory.driveToReefScore(swerve, this);
            // return Commands.parallel(
            // SwerveCommandFactory.driveToReefScore(swerve, this),
            // ElevatorCommandFactory.toPosition(elevator, this).beforeStarting(
            // Commands.waitUntil(elevator::isWithinRadius).withTimeout(2)
            // )
            // );
        }

        public PathPlannerPath getInnerPath() {
            return m_innerPath;
        }

        public ReefFace getFace() {
            return m_face;
        }

        public int getNum() {
            return m_num;
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
    public static final double kReefBranchDistMeters = Conversions.inchesToMeters(13);
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
