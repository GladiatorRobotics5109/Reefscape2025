package frc.robot.util;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ElevatorCommandFactory;
import frc.robot.commands.EndEffectorCommandFactory;
import frc.robot.commands.SuperstructureCommandFactory;
import frc.robot.commands.SwerveCommandFactory;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class FieldConstants {
    public static final class ReefConstants {
        public static enum ReefIndex {
            One,
            Two;

            public int getIndex() {
                return this == One ? 1 : 2;
            }

            @Override
            public String toString() {
                return Integer.toString(getIndex());
            }
        }

        public static enum ReefHeight {
            L1(0.0), // TOOD: Fix this
            L2(kReefL2HeightMeters),
            L3(kReefL3HeightMeters),
            L4(kReefL4HeightMeters);

            private final double m_height;

            private ReefHeight(double heightMeters) {
                m_height = heightMeters;
            }

            public double getHeight() {
                return m_height;
            }
        }

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
                Rotation2d angle = Rotation2d.fromDegrees(60 * -m_index).plus(offset);
                return angle;
            }

            public Rotation2d getFaceAngleFieldRelativeBlueAlliance() {
                return Rotation2d.fromDegrees(60 * -m_index).plus(Rotation2d.k180deg);
            }

            public Rotation2d getSwerveTargetHeading() {
                return getFaceAngleFieldRelative().plus(Rotation2d.k180deg);
            }

            public Rotation2d getSwerveTargetHeadingBlueAlliance() {
                return getFaceAngleFieldRelativeBlueAlliance().plus(Rotation2d.k180deg);
            }

            public Translation2d getFieldRelativeFacePosition() {
                return new Translation2d(kReefRadiusMeters, getFaceAngleFieldRelative()).plus(getAllianceReefPos());
            }

            public Translation2d getFieldRelativeFacePositionBlueAlliance() {
                return new Translation2d(kReefRadiusMeters, getFaceAngleFieldRelativeBlueAlliance()).plus(kBlueReefPos);
            }

            public Translation2d getReefRelativeFacePosition() {
                return new Translation2d(kReefRadiusMeters, getFaceAngleFieldRelative());
            }

            public int getIndex() {
                return m_index;
            }
        }

        public static class ReefBranch {
            public static final ReefBranch kL2E1 = new ReefBranch(ReefHeight.L2, ReefFace.E, ReefIndex.One);
            public static final ReefBranch kL2E2 = new ReefBranch(ReefHeight.L2, ReefFace.E, ReefIndex.Two);
            public static final ReefBranch kL2F1 = new ReefBranch(ReefHeight.L2, ReefFace.F, ReefIndex.One);
            public static final ReefBranch kL2F2 = new ReefBranch(ReefHeight.L2, ReefFace.F, ReefIndex.Two);
            public static final ReefBranch kL2G1 = new ReefBranch(ReefHeight.L2, ReefFace.G, ReefIndex.One);
            public static final ReefBranch kL2G2 = new ReefBranch(ReefHeight.L2, ReefFace.G, ReefIndex.Two);
            public static final ReefBranch kL2H1 = new ReefBranch(ReefHeight.L2, ReefFace.H, ReefIndex.One);
            public static final ReefBranch kL2H2 = new ReefBranch(ReefHeight.L2, ReefFace.H, ReefIndex.Two);
            public static final ReefBranch kL2I1 = new ReefBranch(ReefHeight.L2, ReefFace.I, ReefIndex.One);
            public static final ReefBranch kL2I2 = new ReefBranch(ReefHeight.L2, ReefFace.I, ReefIndex.Two);
            public static final ReefBranch kL2J1 = new ReefBranch(ReefHeight.L2, ReefFace.J, ReefIndex.One);
            public static final ReefBranch kL2J2 = new ReefBranch(ReefHeight.L2, ReefFace.J, ReefIndex.Two);

            public static final ReefBranch kL3E1 = new ReefBranch(ReefHeight.L3, ReefFace.E, ReefIndex.One);
            public static final ReefBranch kL3E2 = new ReefBranch(ReefHeight.L3, ReefFace.E, ReefIndex.Two);
            public static final ReefBranch kL3F1 = new ReefBranch(ReefHeight.L3, ReefFace.F, ReefIndex.One);
            public static final ReefBranch kL3F2 = new ReefBranch(ReefHeight.L3, ReefFace.F, ReefIndex.Two);
            public static final ReefBranch kL3G1 = new ReefBranch(ReefHeight.L3, ReefFace.G, ReefIndex.One);
            public static final ReefBranch kL3G2 = new ReefBranch(ReefHeight.L3, ReefFace.G, ReefIndex.Two);
            public static final ReefBranch kL3H1 = new ReefBranch(ReefHeight.L3, ReefFace.H, ReefIndex.One);
            public static final ReefBranch kL3H2 = new ReefBranch(ReefHeight.L3, ReefFace.H, ReefIndex.Two);
            public static final ReefBranch kL3I1 = new ReefBranch(ReefHeight.L3, ReefFace.I, ReefIndex.One);
            public static final ReefBranch kL3I2 = new ReefBranch(ReefHeight.L3, ReefFace.I, ReefIndex.Two);
            public static final ReefBranch kL3J1 = new ReefBranch(ReefHeight.L3, ReefFace.J, ReefIndex.One);
            public static final ReefBranch kL3J2 = new ReefBranch(ReefHeight.L3, ReefFace.J, ReefIndex.Two);

            public static final ReefBranch kL4E1 = new ReefBranch(ReefHeight.L4, ReefFace.E, ReefIndex.One);
            public static final ReefBranch kL4E2 = new ReefBranch(ReefHeight.L4, ReefFace.E, ReefIndex.Two);
            public static final ReefBranch kL4F1 = new ReefBranch(ReefHeight.L4, ReefFace.F, ReefIndex.One);
            public static final ReefBranch kL4F2 = new ReefBranch(ReefHeight.L4, ReefFace.F, ReefIndex.Two);
            public static final ReefBranch kL4G1 = new ReefBranch(ReefHeight.L4, ReefFace.G, ReefIndex.One);
            public static final ReefBranch kL4G2 = new ReefBranch(ReefHeight.L4, ReefFace.G, ReefIndex.Two);
            public static final ReefBranch kL4H1 = new ReefBranch(ReefHeight.L4, ReefFace.H, ReefIndex.One);
            public static final ReefBranch kL4H2 = new ReefBranch(ReefHeight.L4, ReefFace.H, ReefIndex.Two);
            public static final ReefBranch kL4I1 = new ReefBranch(ReefHeight.L4, ReefFace.I, ReefIndex.One);
            public static final ReefBranch kL4I2 = new ReefBranch(ReefHeight.L4, ReefFace.I, ReefIndex.Two);
            public static final ReefBranch kL4J1 = new ReefBranch(ReefHeight.L4, ReefFace.J, ReefIndex.One);
            public static final ReefBranch kL4J2 = new ReefBranch(ReefHeight.L4, ReefFace.J, ReefIndex.Two);

            private final Translation3d m_branchPosition;
            private final PathPlannerPath m_innerPath;
            private final ReefFace m_face;
            private final ReefHeight m_height;
            private final ReefIndex m_index;

            public ReefBranch(ReefHeight height, ReefFace face, ReefIndex index) {
                m_face = face;
                m_height = height;
                m_index = index;

                // I'm very sorry for this...
                m_branchPosition = new Translation3d(getAllianceReefPos()).plus(
                    new Translation3d(face.getReefRelativeFacePosition())
                ).plus(
                    new Translation3d(
                        kReefBranchDistMeters / 2,
                        new Rotation3d(
                            m_face.getFaceAngleFieldRelative().plus(
                                Util.getAlliance() == Alliance.Blue
                                    ? (m_index == ReefIndex.One ? Rotation2d.fromDegrees(90)
                                        : Rotation2d.fromDegrees(-90))
                                    : (m_index == ReefIndex.One ? Rotation2d.fromDegrees(-90)
                                        : Rotation2d.fromDegrees(90))
                            )
                        )
                    )
                ).plus(new Translation3d(0, 0, height.getHeight()));

                m_innerPath = Paths.getReefInnerPath(face, index);
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
                return new Pose2d(
                    waypoints.get(waypoints.size() - 1).anchor(),
                    m_innerPath.getGoalEndState().rotation()
                );
            }

            public Translation3d getBranchPosition() {
                return m_branchPosition;
            }

            public
                Command
                makeScoreCommand(SwerveSubsystem swerve, ElevatorSubsystem elevator, EndEffectorSubsystem endEffector) {
                // return SwerveCommandFactory.driveToReefScore(swerve, this);
                return Commands.parallel(
                    SwerveCommandFactory.driveToReefScore(swerve, this),
                    SuperstructureCommandFactory.autoScore(elevator, endEffector, this)
                ).withName(this + " Score Command");
            }

            public PathPlannerPath getInnerPath() {
                return m_innerPath;
            }

            public ReefHeight getHeight() {
                return m_height;
            }

            public ReefFace getFace() {
                return m_face;
            }

            public ReefIndex getIndex() {
                return m_index;
            }

            @Override
            public String toString() {
                return m_height.toString() + m_face.toString() + m_index.getIndex();
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

        public static Translation2d getAllianceReefPos() {
            Logger.recordOutput(
                "AllianceReefPose",
                new Translation3d(kBlueReefPos.getX(), kBlueReefPos.getY(), 2.872660334721429)
            );
            return Util.getAlliance() == Alliance.Red ? kRedReefPos : kBlueReefPos;
        }
    }

    public static final class CoralStationConstants {
        public static enum CoralStationIndex {
            One(1),
            Two(2),
            Three(3);

            private final int m_index;

            private CoralStationIndex(int index) {
                m_index = index;
            }

            public int getIndex() {
                return m_index;
            }

            @Override
            public String toString() {
                return Integer.toString(getIndex());
            }
        }

        public static enum CoralStationSide {
            C, // Close
            F // Far
        }

        public static final class CoralStation {
            public static final CoralStation kC1 = new CoralStation(CoralStationSide.C, CoralStationIndex.One);
            public static final CoralStation kC2 = new CoralStation(CoralStationSide.C, CoralStationIndex.Two);
            public static final CoralStation kC3 = new CoralStation(CoralStationSide.C, CoralStationIndex.Three);

            public static final CoralStation kF1 = new CoralStation(CoralStationSide.F, CoralStationIndex.One);
            public static final CoralStation kF2 = new CoralStation(CoralStationSide.F, CoralStationIndex.Two);
            public static final CoralStation kF3 = new CoralStation(CoralStationSide.F, CoralStationIndex.Three);

            private final CoralStationSide m_side;
            private final CoralStationIndex m_index;

            private final Translation2d m_position;

            public CoralStation(CoralStationSide side, CoralStationIndex index) {
                m_side = side;
                m_index = index;
                Translation2d indexOffset = new Translation2d(
                    (2 - m_index.getIndex()) * (kCoralStationOpeningWidthMeters / 3),
                    getAllianceSideCoralStationFaceAngle(side).plus(Rotation2d.fromDegrees(90))
                );
                m_position = getBlueCoralStationPos(side).getTranslation().plus(indexOffset);
            }

            public Translation2d getPosition() {
                return m_position;
            }

            public Rotation2d getFaceAngle() {
                return getAllianceSideCoralStationFaceAngle(m_side);
            }

            public CoralStationSide getSide() {
                return m_side;
            }

            public CoralStationIndex getIndex() {
                return m_index;
            }

            public PathPlannerPath getInnerPath() {
                return Paths.getCoralStationInnerPath(this);
            }

            public Command makeIntakeCommand(
                SwerveSubsystem swerve,
                ElevatorSubsystem elevator,
                EndEffectorSubsystem endEffector
            ) {
                return Commands.parallel(
                    SwerveCommandFactory.driveToPoseThenFollowPath(
                        SwerveConstants.kPPPathFindConstraints,
                        getInnerPath()
                    ),
                    ElevatorCommandFactory.toHome(elevator),
                    EndEffectorCommandFactory.intake(endEffector)
                );
            }

            @Override
            public String toString() {
                return m_side.toString() + m_index.toString();
            }
        }

        public static final double kCoralStationOpeningWidthMeters = Conversions.feetToMeters(6)
            + Conversions.inchesToMeters(4);
        public static final double kCoralStationOpeningHeightMeters = Conversions.inchesToMeters(7);
        public static final double kCoralStationOpeningCarpetHeightMeters = Conversions.feetToMeters(3)
            + Conversions.inchesToMeters(1.5);

        public static final Rotation2d kCoralStationChuteAngle = Rotation2d.fromDegrees(55);

        public static final Rotation2d kCoralStationCloseFaceAngle = Rotation2d.fromDegrees(54);
        public static final Rotation2d kCoralStationFarFaceAngle = Rotation2d.fromDegrees(-54);

        public static final Pose2d kBlueCloseCoralStationPose = new Pose2d(
            Conversions.inchesToMeters(33.526),
            Conversions.inchesToMeters(25.824),
            kCoralStationCloseFaceAngle
        );
        public static final Pose2d kBlueFarCoralStationPose = new Pose2d(
            Conversions.inchesToMeters(33.526),
            Conversions.inchesToMeters(291.176),
            kCoralStationFarFaceAngle
        );

        public static final Pose2d getBlueCoralStationPos(CoralStationSide side) {
            return side == CoralStationSide.C ? kBlueCloseCoralStationPose : kBlueFarCoralStationPose;
        }

        // public static final Pose2d getAllianceCoralStationPos(CoralStationSide side) {
        // return Util.getAlliance() == Alliance.Blue
        // ? (side == CoralStationSide.C ? kBlueCloseCoralStationPos : kBlueFarCoralStationPos)
        // : (side == CoralStationSide.C ? kRedCloseCoralStationPos : kRedFarCoralStationPos);
        // }

        public static final Rotation2d getAllianceSideCoralStationFaceAngle(CoralStationSide side) {
            Rotation2d offset = Util.getAlliance() == Alliance.Blue ? Rotation2d.fromDegrees(0)
                : Rotation2d.fromDegrees(90);
            Rotation2d angle = side == CoralStationSide.C ? kCoralStationCloseFaceAngle : kCoralStationFarFaceAngle;

            return angle.plus(offset);
        }

        public static final Rotation2d getCoralStationFaceAngleBlueAlliance(CoralStationSide side) {
            return side == CoralStationSide.C ? kCoralStationCloseFaceAngle : kCoralStationFarFaceAngle;
        }
    }
}
