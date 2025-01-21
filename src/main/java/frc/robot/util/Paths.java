package frc.robot.util;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.FieldConstants.CoralStationConstants;
import frc.robot.util.FieldConstants.CoralStationConstants.CoralStation;
import frc.robot.util.FieldConstants.CoralStationConstants.CoralStationIndex;
import frc.robot.util.FieldConstants.CoralStationConstants.CoralStationSide;
import frc.robot.util.FieldConstants.ReefConstants.ReefFace;
import frc.robot.util.FieldConstants.ReefConstants.ReefIndex;

public final class Paths {
    private static record ReefFaceAndIndex(ReefFace face, ReefIndex index) {
        @Override
        public String toString() {
            return face.toString() + index.toString();
        }
    }

    private static Map<ReefFaceAndIndex, PathPlannerPath> s_reefPaths = new HashMap<>();
    private static Map<CoralStation, PathPlannerPath> s_coralPaths = new HashMap<>();

    public static Pose2d[][] generatedReefPaths = new Pose2d[12][0];
    public static Pose2d[][] generatedCoralStationPaths = new Pose2d[6][];

    private static boolean s_hasInit = false;

    public static void init() {
        if (s_hasInit)
            return;

        // A lil jank but it works so
        // I'll make this happen during compile-time a some point too
        ReefFace[] faces = ReefFace.values();
        for (ReefFace face : faces) {
            Rotation2d swerveTargetHeading = face.getSwerveTargetHeadingBlueAlliance();
            Translation2d faceLocation = face.getFieldRelativeFacePositionBlueAlliance();

            if (face == ReefFace.E || face == ReefFace.I || face == ReefFace.J) {
                s_reefPaths.put(
                    new ReefFaceAndIndex(face, ReefIndex.One),
                    generateFromPosition(
                        faceLocation.plus(
                            new Translation2d(
                                FieldConstants.ReefConstants.kReefBranchDistMeters / 2,
                                swerveTargetHeading.plus(Rotation2d.fromDegrees(90))
                            )
                        ),
                        swerveTargetHeading
                    )
                );

                s_reefPaths.put(
                    new ReefFaceAndIndex(face, ReefIndex.Two),
                    generateFromPosition(
                        faceLocation.plus(
                            new Translation2d(
                                FieldConstants.ReefConstants.kReefBranchDistMeters / 2,
                                swerveTargetHeading.minus(Rotation2d.fromDegrees(90))
                            )
                        ),
                        swerveTargetHeading
                    )
                );
            }
            else {
                s_reefPaths.put(
                    new ReefFaceAndIndex(face, ReefIndex.One),
                    generateFromPosition(
                        faceLocation.plus(
                            new Translation2d(
                                FieldConstants.ReefConstants.kReefBranchDistMeters / 2,
                                swerveTargetHeading.minus(Rotation2d.fromDegrees(90))
                            )
                        ),
                        swerveTargetHeading
                    )
                );

                s_reefPaths.put(
                    new ReefFaceAndIndex(face, ReefIndex.Two),
                    generateFromPosition(
                        faceLocation.plus(
                            new Translation2d(
                                FieldConstants.ReefConstants.kReefBranchDistMeters / 2,
                                swerveTargetHeading.plus(Rotation2d.fromDegrees(90))
                            )
                        ),
                        swerveTargetHeading
                    )
                );
            }
        }

        CoralStationSide[] sides = CoralStationSide.values();
        for (CoralStationSide side : sides) {
            CoralStationIndex[] indexes = CoralStationIndex.values();

            for (CoralStationIndex index : indexes) {
                CoralStation station = new CoralStation(side, index);
                s_coralPaths.put(
                    station,
                    generateFromPosition(
                        (side == CoralStationSide.C ? CoralStationConstants.kBlueCloseCoralStationPos
                            : CoralStationConstants.kBlueFarCoralStationPos).plus(
                                new Translation2d(
                                    (2 - index.getIndex())
                                        * (CoralStationConstants.kCoralStationOpeningWidthMeters / 3),
                                    CoralStationConstants.getBlueSideCoralStationFaceAngle(side).plus(
                                        Rotation2d.fromDegrees(90)
                                    )
                                )
                            ),
                        CoralStationConstants.getBlueSideCoralStationFaceAngle(side).plus(Rotation2d.fromDegrees(180))
                    )
                );
            }
        }

        s_hasInit = true;
    }

    // Probably no need to do this bc the path will be flipped dpepending on alliance
    public static void regenerate() {
        s_hasInit = false;
        init();
    }

    public static boolean hasInit() {
        return s_hasInit;
    }

    public static PathPlannerPath getReefInnerPath(ReefFace face, ReefIndex i) {
        if (!s_hasInit)
            init();

        return s_reefPaths.get(new ReefFaceAndIndex(face, i));
    }

    // This won't work if its called before Logger.start()
    // the init() method will probably be called before this
    // that's why this should be called manually
    public static void log() {
        if (!s_hasInit)
            init();

        final String kLogPath = "Paths";
        final String kLogPathReef = kLogPath + "/ReefPaths";
        final String kLogPathCoral = kLogPath + "/CoralStationPaths";

        s_reefPaths.forEach(
            (faceAndIndex, path) -> Logger.recordOutput(
                kLogPathReef + "/" + faceAndIndex.toString(),
                path.getPathPoses().toArray(new Pose2d[0])
            )
        );
        s_coralPaths.forEach(
            (station, path) -> Logger.recordOutput(
                kLogPathCoral + "/" + station.toString(),
                path.getPathPoses().toArray(new Pose2d[0])
            )
        );
    }

    /**
     * Generates a straight path given a position and target heading for the drivetrain
     *
     * @param pos
     *            Goal end position
     * @param swerveTargetHeading
     *            Swerve target heading
     */
    private static PathPlannerPath generateFromPosition(Translation2d pos, Rotation2d swerveTargetHeading) {
        Translation2d targetEndPos = pos.plus(
            new Translation2d(
                Constants.SwerveConstants.kFrameHeight / 2 + Constants.kBumperWidthMeters,
                swerveTargetHeading.plus(Rotation2d.fromDegrees(180))
            )
        );
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(
                targetEndPos.plus(new Translation2d(0.8, swerveTargetHeading.plus(Rotation2d.k180deg))),
                swerveTargetHeading
            ),
            new Pose2d(targetEndPos, swerveTargetHeading)
        );

        return new PathPlannerPath(
            waypoints,
            SwerveConstants.kPPReefInnerPathConstraints,
            new IdealStartingState(
                SwerveConstants.kPPReefInnerPathConstraints.maxVelocity(),
                swerveTargetHeading
            ),
            new GoalEndState(0, swerveTargetHeading)
        );
    }
}
