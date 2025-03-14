package frc.robot.util;

import java.util.List;
import java.util.Optional;

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

public class FieldConstants {
    public static final class ReefConstants {
        public static enum ReefIndex {
            One,
            Two;

            public static Optional<ReefIndex> fromChar(char c) {
                switch (c) {
                    case '1':
                        return Optional.of(One);
                    case '2':
                        return Optional.of(Two);
                    default:
                        return Optional.empty();
                }
            }

            public int getIndex() { return this == One ? 1 : 2; }

            @Override
            public String toString() {
                return Integer.toString(getIndex());
            }
        }

        public static enum ReefHeight {
            L1(kReefL1HeightMeters),
            L2(kReefL2HeightMeters),
            L3(kReefL3HeightMeters),
            L4(kReefL4HeightMeters);

            public static Optional<ReefHeight> fromChar(char c) {
                switch (c) {
                    case '1':
                        return Optional.of(L1);
                    case '2':
                        return Optional.of(L2);
                    case '3':
                        return Optional.of(L3);
                    case '4':
                        return Optional.of(L4);
                    default:
                        return Optional.empty();
                }
            }

            private final double m_height;

            private ReefHeight(double heightMeters) {
                m_height = heightMeters;
            }

            public double getHeight() { return m_height; }
        }

        public static enum ReefFace {
            E(0),
            F(1),
            G(2),
            H(3),
            I(4),
            J(5);

            public static Optional<ReefFace> fromChar(char c) {
                switch (c) {
                    case 'E':
                        return Optional.of(E);
                    case 'F':
                        return Optional.of(F);
                    case 'G':
                        return Optional.of(G);
                    case 'H':
                        return Optional.of(H);
                    case 'I':
                        return Optional.of(I);
                    case 'J':
                        return Optional.of(J);
                    default:
                        return Optional.empty();
                }
            }

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

            public Rotation2d getSwerveTargetHeading() { return getFaceAngleFieldRelative().plus(Rotation2d.k180deg); }

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

            public int getIndex() { return m_index; }
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

            public static final ReefBranch[] kValues = new ReefBranch[] {
                ReefBranch.kL2E1,
                ReefBranch.kL2E2,
                ReefBranch.kL2F1,
                ReefBranch.kL2F2,
                ReefBranch.kL2G1,
                ReefBranch.kL2G2,
                ReefBranch.kL2H1,
                ReefBranch.kL2H2,
                ReefBranch.kL2I1,
                ReefBranch.kL2I2,
                ReefBranch.kL2J1,
                ReefBranch.kL2J2,

                ReefBranch.kL3E1,
                ReefBranch.kL3E2,
                ReefBranch.kL3F1,
                ReefBranch.kL3F2,
                ReefBranch.kL3G1,
                ReefBranch.kL3G2,
                ReefBranch.kL3H1,
                ReefBranch.kL3H2,
                ReefBranch.kL3I1,
                ReefBranch.kL3I2,
                ReefBranch.kL3J1,
                ReefBranch.kL3J2,

                ReefBranch.kL4E1,
                ReefBranch.kL4E2,
                ReefBranch.kL4F1,
                ReefBranch.kL4F2,
                ReefBranch.kL4G1,
                ReefBranch.kL4G2,
                ReefBranch.kL4H1,
                ReefBranch.kL4H2,
                ReefBranch.kL4I1,
                ReefBranch.kL4I2,
                ReefBranch.kL4J1,
                ReefBranch.kL4J2,
            };

            public static final Optional<ReefBranch> fromString(String str) {
                Optional<ReefHeight> height = ReefHeight.fromChar(str.charAt(1));
                Optional<ReefFace> face = ReefFace.fromChar(str.charAt(2));
                Optional<ReefIndex> index = ReefIndex.fromChar(str.charAt(3));

                if (height.isEmpty() || face.isEmpty() || index.isEmpty()) return Optional.empty();

                return Optional.of(new ReefBranch(height.get(), face.get(), index.get()));
            }

            private final Translation3d m_branchPosition;
            private final PathPlannerPath m_innerPath;
            private final ReefFace m_face;
            private final ReefHeight m_height;
            private final ReefIndex m_index;

            public ReefBranch(ReefHeight height, ReefFace face, ReefIndex index) {
                m_face = face;
                m_height = height;
                m_index = index;

                m_branchPosition = getBranchPosition(m_height, m_face, m_index);

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

            public Translation3d getBranchPosition() { return m_branchPosition; }

            public PathPlannerPath getInnerPath() { return m_innerPath; }

            public ReefHeight getHeight() { return m_height; }

            public ReefFace getFace() { return m_face; }

            public ReefIndex getIndex() { return m_index; }

            @Override
            public String toString() {
                return m_height.toString() + m_face.toString() + m_index.getIndex();
            }

            @Override
            public boolean equals(Object o) {
                if (o == null) return false;
                if (o.getClass() != this.getClass()) return false;

                final ReefBranch other = (ReefBranch)o;
                return this.getBranchPosition().equals(other.getBranchPosition());
            }

            private Translation3d getBranchPosition(ReefHeight height, ReefFace face, ReefIndex index) {
                // Face position
                Translation3d position = new Translation3d(getAllianceReefPos()).plus(
                    new Translation3d(face.getReefRelativeFacePosition())
                ).plus(new Translation3d(0.0, 0.0, height.getHeight()));

                Rotation2d toIndex = face.getFaceAngleFieldRelative();
                if (face == ReefFace.E || face == ReefFace.I || face == ReefFace.J) {
                    toIndex = toIndex.plus(index == ReefIndex.One ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg);
                }
                else {
                    toIndex = toIndex.plus(index == ReefIndex.One ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg);
                }

                position = position.plus(new Translation3d(kReefBranchDistMeters / 2, new Rotation3d(toIndex)));

                return position;
            }
        }

        public static final double kReefDiameterMeters = Conversions.inchesToMeters(65.5);
        public static final double kReefRadiusMeters = kReefDiameterMeters / 2;
        // Distance form reef base to branch at L2 and L3
        public static final double kReefL2L3InsetMeters = Conversions.inchesToMeters(1 + (5 / 8));
        public static final double kReefL4InsetMeters = Conversions.inchesToMeters(1 + (1 / 8));

        public static final double kReefL1HeightMeters = Conversions.feetToMeters(1) + Conversions.inchesToMeters(6);
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

            public int getIndex() { return m_index; }

            @Override
            public String toString() {
                return Integer.toString(getIndex());
            }

            public static Optional<CoralStationIndex> fromChar(char c) {
                return switch (c) {
                    case '1' -> Optional.of(One);
                    case '2' -> Optional.of(Two);
                    case '3' -> Optional.of(Three);
                    default -> Optional.empty();
                };
            }
        }

        public static enum CoralStationSide {
            C, // Close
            F; // Far

            public static Optional<CoralStationSide> fromChar(char c) {
                return switch (c) {
                    case 'C' -> Optional.of(C);
                    case 'F' -> Optional.of(F);
                    default -> Optional.empty();
                };
            }
        }

        public static final class CoralStation {
            public static final CoralStation kC1 = new CoralStation(CoralStationSide.C, CoralStationIndex.One);
            public static final CoralStation kC2 = new CoralStation(CoralStationSide.C, CoralStationIndex.Two);
            public static final CoralStation kC3 = new CoralStation(CoralStationSide.C, CoralStationIndex.Three);

            public static final CoralStation kF1 = new CoralStation(CoralStationSide.F, CoralStationIndex.One);
            public static final CoralStation kF2 = new CoralStation(CoralStationSide.F, CoralStationIndex.Two);
            public static final CoralStation kF3 = new CoralStation(CoralStationSide.F, CoralStationIndex.Three);

            public static final CoralStation[] kValues = new CoralStation[] {
                CoralStation.kC1,
                CoralStation.kC2,
                CoralStation.kC3,

                CoralStation.kF1,
                CoralStation.kF2,
                CoralStation.kF3,
            };

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

            public Translation2d getPosition() { return m_position; }

            public Rotation2d getFaceAngle() { return getAllianceSideCoralStationFaceAngle(m_side); }

            public CoralStationSide getSide() { return m_side; }

            public CoralStationIndex getIndex() { return m_index; }

            public PathPlannerPath getInnerPath() { return Paths.getCoralStationInnerPath(this); }

            @Override
            public String toString() {
                return m_side.toString() + m_index.toString();
            }

            @Override
            public boolean equals(Object o) {
                if (o == null) return false;
                if (o.getClass() != this.getClass()) return false;

                final CoralStation other = (CoralStation)o;
                return this.getPosition().equals(other.getPosition());
            }

            public static Optional<CoralStation> fromString(String str) {
                Optional<CoralStationSide> side = CoralStationSide.fromChar(str.charAt(0));
                Optional<CoralStationIndex> index = CoralStationIndex.fromChar(str.charAt(1));

                if (side.isEmpty() || index.isEmpty()) return Optional.empty();

                return Optional.of(new CoralStation(side.get(), index.get()));
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
            Rotation2d offset = Util.getAlliance() == Alliance.Blue
                ? Rotation2d.fromDegrees(0)
                : Rotation2d.fromDegrees(90);
            Rotation2d angle = side == CoralStationSide.C ? kCoralStationCloseFaceAngle : kCoralStationFarFaceAngle;

            return angle.plus(offset);
        }

        public static final Rotation2d getCoralStationFaceAngleBlueAlliance(CoralStationSide side) {
            return side == CoralStationSide.C ? kCoralStationCloseFaceAngle : kCoralStationFarFaceAngle;
        }
    }

    public static final class CoralConstants {
        public static final double kCoralLengthMeters = Conversions.inchesToMeters(11 + (7 / 8));

        public static final double kCoralInnerDiameterMeters = Conversions.inchesToMeters(4);
        public static final double kCoralInnerRadiusMeters = kCoralInnerDiameterMeters / 2;

        public static final double kCoralOuterDiameterMeters = kCoralInnerDiameterMeters
            + Conversions.inchesToMeters(0.5);
        public static final double kCoralOuterRadiusMeters = kCoralOuterDiameterMeters / 2;
    }
}
