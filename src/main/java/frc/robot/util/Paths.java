package frc.robot.util;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.FieldConstants.ReefFace;
import frc.robot.util.FieldConstants.ReefIndex;

public final class Paths {
    private static record FaceAndIndex(ReefFace face, ReefIndex index) {}

    private static Map<FaceAndIndex, PathPlannerPath> s_paths = new HashMap<FaceAndIndex, PathPlannerPath>();

    public static Pose2d[][] test = new Pose2d[12][0];

    private static boolean s_hasInit = false;

    public static void init() {
        if (s_hasInit)
            return;

        ReefFace[] faces = ReefFace.values();
        int i = 0;
        for (ReefFace face : faces) {
            Rotation2d swerveTargetHeading = face.getSwerveTargetHeadingBlueAlliance();
            Rotation2d faceAngle = swerveTargetHeading.plus(Rotation2d.k180deg);
            Translation2d faceLocation = face.getFieldRelativeFacePositionBlueAlliance();

            s_paths.put(
                new FaceAndIndex(face, ReefIndex.One),
                generateFromPosition(
                    faceLocation.plus(
                        new Translation2d(
                            FieldConstants.kReefBranchDistMeters / 2,
                            swerveTargetHeading.plus(Rotation2d.fromDegrees(90))
                        )
                    ),
                    swerveTargetHeading
                )
            );
            test[i] = generateFromPosition(
                faceLocation.plus(
                    new Translation2d(
                        FieldConstants.kReefBranchDistMeters / 2,
                        swerveTargetHeading.plus(Rotation2d.fromDegrees(90))
                    )
                ),
                swerveTargetHeading
            ).getPathPoses().toArray(new Pose2d[0]);
            i++;

            s_paths.put(
                new FaceAndIndex(face, ReefIndex.Two),
                generateFromPosition(
                    faceLocation.plus(
                        new Translation2d(
                            FieldConstants.kReefBranchDistMeters / 2,
                            swerveTargetHeading.minus(Rotation2d.fromDegrees(90))
                        )
                    ),
                    swerveTargetHeading
                )
            );

            test[i] = generateFromPosition(
                faceLocation.plus(
                    new Translation2d(
                        FieldConstants.kReefBranchDistMeters / 2,
                        swerveTargetHeading.minus(Rotation2d.fromDegrees(90))
                    )
                ),
                swerveTargetHeading
            ).getPathPoses().toArray(new Pose2d[0]);
            i++;
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

        return s_paths.get(new FaceAndIndex(face, i));
    }

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
