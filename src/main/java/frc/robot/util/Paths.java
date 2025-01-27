package frc.robot.util;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
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

    private static record CoralStationSideAndIndex(CoralStationSide side, CoralStationIndex index) {
        @Override
        public String toString() {
            return side.toString() + index.toString();
        }
    }

    private static Map<ReefFaceAndIndex, PathPlannerPath> s_reefPaths = new HashMap<>();
    private static Map<CoralStationSideAndIndex, PathPlannerPath> s_coralPaths = new HashMap<>();

    private static boolean s_hasInit = false;

    public static void init() {
        if (s_hasInit)
            return;

        // A lil jank but it works so...
        // I'll make this happen during compile-time a some point too
        ReefFace[] faces = ReefFace.values();
        for (ReefFace face : faces) {
            Rotation2d faceAngle = face.getFaceAngleFieldRelativeBlueAlliance();
            Translation2d faceLocation = face.getFieldRelativeFacePositionBlueAlliance();

            s_reefPaths.put(
                new ReefFaceAndIndex(face, ReefIndex.One),
                generateReefInnerPath(
                    faceLocation,
                    face.getFaceAngleFieldRelativeBlueAlliance(),
                    face == ReefFace.E || face == ReefFace.I || face == ReefFace.J
                        ? Rotation2d.kCW_90deg
                        : Rotation2d.kCCW_90deg
                )
            );

            s_reefPaths.put(
                new ReefFaceAndIndex(face, ReefIndex.Two),
                generateReefInnerPath(
                    faceLocation,
                    faceAngle,
                    face == ReefFace.E || face == ReefFace.I || face == ReefFace.J
                        ? Rotation2d.kCCW_90deg
                        : Rotation2d.kCW_90deg
                )
            );
        }

        CoralStationSide[] sides = CoralStationSide.values();
        for (CoralStationSide side : sides) {
            CoralStationIndex[] indexes = CoralStationIndex.values();
            for (CoralStationIndex index : indexes) {
                s_coralPaths.put(new CoralStationSideAndIndex(side, index), generateCoralStationInnerPath(side, index));
            }
        }

        s_hasInit = true;
    }

    public static boolean hasInit() {
        return s_hasInit;
    }

    public static PathPlannerPath getReefInnerPath(ReefFace face, ReefIndex i) {
        if (!s_hasInit)
            init();

        return s_reefPaths.get(new ReefFaceAndIndex(face, i));
    }

    public static PathPlannerPath getCoralStationInnerPath(CoralStation station) {
        if (!s_hasInit)
            init();

        return s_coralPaths.get(new CoralStationSideAndIndex(station.getSide(), station.getIndex()));
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

    private static PathPlannerPath generateReefInnerPath(
        Translation2d facePos,
        Rotation2d faceAngle,
        Rotation2d toBranch
    ) {
        Translation2d branchPos = facePos.plus(
            new Translation2d(FieldConstants.ReefConstants.kReefBranchDistMeters / 2, faceAngle.plus(toBranch))
        );

        return generatePathAlongVector(
            branchPos.plus(
                new Translation2d(SwerveConstants.kFrameHeight / 2 + Constants.kBumperWidthMeters, faceAngle)
            ),
            new Translation2d(0.8, faceAngle),
            faceAngle.plus(Rotation2d.k180deg),
            SwerveConstants.kPPReefInnerPathConstraints
        );
    }

    private static PathPlannerPath generateCoralStationInnerPath(CoralStationSide side, CoralStationIndex index) {
        Translation2d stationPos = side == CoralStationSide.C
            ? CoralStationConstants.kBlueCloseCoralStationPose.getTranslation()
            : CoralStationConstants.kBlueFarCoralStationPose.getTranslation();
        Rotation2d faceAngle = CoralStationConstants.getCoralStationFaceAngleBlueAlliance(side);

        Translation2d facePos = stationPos.plus(
            new Translation2d(
                (2 - index.getIndex()) * (CoralStationConstants.kCoralStationOpeningWidthMeters / 3),
                faceAngle.plus(side == CoralStationSide.C ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg)
            )
        );

        return generatePathAlongVector(
            facePos.plus(new Translation2d(SwerveConstants.kFrameHeight / 2 + Constants.kBumperWidthMeters, faceAngle)),
            new Translation2d(0.8, faceAngle),
            faceAngle,
            SwerveConstants.kPPCoralStationInnerPathConstraints
        );
    }

    /**
     *
     * @param endPos
     * @param direction
     * @param targetHeading
     * @param constraints
     *
     * @return
     */
    private static PathPlannerPath generatePathAlongVector(
        Translation2d endPos,
        Translation2d direction,
        Rotation2d targetHeading,
        PathConstraints constraints
    ) {
        Translation2d startPos = endPos.plus(direction);
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(startPos, targetHeading),
            new Pose2d(endPos, targetHeading)
        );

        return new PathPlannerPath(
            waypoints,
            constraints,
            new IdealStartingState(constraints.maxVelocity(), targetHeading),
            new GoalEndState(0, targetHeading)
        );
    }
}
