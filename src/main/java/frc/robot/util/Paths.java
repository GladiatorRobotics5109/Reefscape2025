package frc.robot.util;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.FieldConstants.ReefFace;

public final class Paths {
    private static PathPlannerPath[] s_paths = new PathPlannerPath[12];
    private static Alliance s_generatedAllaince = Constants.kDefaultAlliance;

    public static Pose2d[] test = new Pose2d[0];

    private static boolean s_hasInit = false;

    public static void init() {
        if (s_hasInit)
            return;

        ReefFace[] faces = ReefFace.values();
        int i = 0;
        for (ReefFace face : faces) {
            Rotation2d swerveTargetHeading = face.getSwerveTargetHeading();
            Translation2d faceLocation = face.getFacePosition();

            s_paths[i] = generateFromPosition(
                faceLocation.plus(
                    new Translation2d(
                        FieldConstants.kReefBranchDistMeters,
                        faceLocation.getAngle().plus(Rotation2d.fromDegrees(90))
                    )
                ),
                swerveTargetHeading
            );
            if (test.length == 0) {
                test = s_paths[i].getPathPoses().toArray(new Pose2d[0]);
            }
            s_paths[i + 1] = generateFromPosition(
                faceLocation.plus(
                    new Translation2d(
                        FieldConstants.kReefBranchDistMeters,
                        faceLocation.getAngle().minus(Rotation2d.fromDegrees(90))
                    )
                ),
                swerveTargetHeading
            );
            i += 2;
        }

        s_generatedAllaince = Util.getAlliance();
        s_hasInit = true;
    }

    public static void regenerate() {
        s_hasInit = false;
        init();
    }

    public static boolean hasInit() {
        return s_hasInit;
    }

    public static PathPlannerPath getReefInnerPath(int index) {
        if (!s_hasInit)
            init();

        return s_paths[index];
    }

    public static Alliance getGeneratedAlliance() {
        return s_generatedAllaince;
    }

    private static PathPlannerPath generateFromPosition(Translation2d pos, Rotation2d swerveTargetHeading) {
        Translation2d targetEndPos = pos.plus(
            new Translation2d(
                Constants.SwerveConstants.kFrameHeight / 2 + Constants.kBumperWidthMeters,
                swerveTargetHeading.plus(Rotation2d.fromDegrees(180))
            )
        );
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            // Should be 1 m away from reef base
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
