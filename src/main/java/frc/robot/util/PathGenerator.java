// package frc.robot.util;

// import java.io.FileWriter;
// import java.io.IOException;
// import java.util.HashMap;
// import java.util.List;
// import java.util.Map;
// import java.util.Map.Entry;

// import org.json.simple.JSONArray;
// import org.json.simple.JSONObject;

// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.IdealStartingState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.RotationTarget;
// import com.pathplanner.lib.path.Waypoint;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.wpilibj.Filesystem;
// import frc.robot.Constants;
// import frc.robot.Constants.SwerveConstants;
// import frc.robot.util.FieldConstants.CoralStationConstants;
// import frc.robot.util.FieldConstants.CoralStationConstants.CoralStation;
// import frc.robot.util.FieldConstants.CoralStationConstants.CoralStationIndex;
// import frc.robot.util.FieldConstants.CoralStationConstants.CoralStationSide;
// import frc.robot.util.FieldConstants.ReefConstants.ReefFace;
// import frc.robot.util.FieldConstants.ReefConstants.ReefIndex;
// import frc.robot.util.Paths.ReefFaceAndIndex;

// public class PathGenerator {
// private static Map<ReefFaceAndIndex, PathPlannerPath> s_reefPaths = new HashMap<>();
// private static Map<CoralStation, PathPlannerPath> s_coralPaths = new HashMap<>();

// @SuppressWarnings("unchecked")
// public static void main(String[] args) {
// // A lil jank but it works so...
// // I'll make this happen during compile-time a some point too
// ReefFace[] faces = ReefFace.values();
// for (ReefFace face : faces) {
// Rotation2d faceAngle = face.getFaceAngleFieldRelativeBlueAlliance();
// Translation2d faceLocation = face.getFieldRelativeFacePositionBlueAlliance();

// s_reefPaths.put(
// new ReefFaceAndIndex(face, ReefIndex.One),
// generateReefInnerPath(
// faceLocation,
// face.getFaceAngleFieldRelativeBlueAlliance(),
// face == ReefFace.E || face == ReefFace.I || face == ReefFace.J ? Rotation2d.kCW_90deg
// : Rotation2d.kCCW_90deg
// )
// );

// s_reefPaths.put(
// new ReefFaceAndIndex(face, ReefIndex.Two),
// generateReefInnerPath(
// faceLocation,
// faceAngle,
// face == ReefFace.E || face == ReefFace.I || face == ReefFace.J ? Rotation2d.kCCW_90deg
// : Rotation2d.kCW_90deg
// )
// );
// }

// CoralStationSide[] sides = CoralStationSide.values();
// for (CoralStationSide side : sides) {
// CoralStationIndex[] indexes = CoralStationIndex.values();
// for (CoralStationIndex index : indexes) {
// CoralStation station = new CoralStation(side, index);
// s_coralPaths.put(
// station,
// generateCoralStationInnerPath(side, index)
// );
// }
// }

// var reefPaths = s_reefPaths.entrySet();
// var coralPaths = s_coralPaths.entrySet();

// for (Entry<ReefFaceAndIndex, PathPlannerPath> path : reefPaths) {
// String filePath = Filesystem.getDeployDirectory() + "/generated_" + path.getKey().toString();
// JSONObject json = new JSONObject();

// json.put("version", "2025.0");
// json.put("waypoints", waypointsToJson(path.getValue().getWaypoints()));
// json.put("rotationTargets", rotationTargetsToJson(path.getValue().getRotationTargets()));
// json.put("constraintZones", new JSONArray());
// json.put("pointTowardsZones", new JSONArray());
// json.put("eventMarkers", new JSONArray());
// json.put("globalConstraints", constraintsToJson(path.getValue().getGlobalConstraints()));
// json.put("goalEndState", goalEndStateToJson(path.getValue().getGoalEndState()));
// json.put("reversed", path.getValue().isReversed());
// json.put("folder", "Generated");
// json.put("idealStartingState", idealStartingStateToJson(path.getValue().getIdealStartingState()));
// json.put("useDefaultConstraints", true);

// try (FileWriter fileWriter = new FileWriter(filePath)) {
// json.writeJSONString(fileWriter);
// }
// catch (IOException e) {
// throw new RuntimeException(e);
// }
// }
// }

// private static JSONObject idealStartingStateToJson(IdealStartingState idealStartingState) {
// JSONObject json = new JSONObject();

// json.put("velocity", idealStartingState.velocityMPS());
// json.put("rotation", idealStartingState.rotation().getDegrees());

// return json;
// }

// private static JSONObject goalEndStateToJson(GoalEndState goalEndState) {
// JSONObject json = new JSONObject();

// json.put("velocity", goalEndState.velocityMPS());
// json.put("rotation", goalEndState.rotation().getDegrees());

// return json;
// }

// private static JSONObject constraintsToJson(PathConstraints constraints) {
// JSONObject obj = new JSONObject();
// obj.put("maxVelocity", constraints.maxVelocity().in(Units.MetersPerSecond));
// obj.put("maxAcceleration", constraints.maxAcceleration().in(Units.MetersPerSecondPerSecond));
// obj.put("maxAngularVelocity", constraints.maxAngularVelocity().in(Units.RadiansPerSecond));
// obj.put("maxAngularAcceleration", constraints.maxAngularAcceleration().in(Units.RadiansPerSecondPerSecond));
// obj.put("nominalVoltage", constraints.nominalVoltageVolts());
// obj.put("unlimited", constraints.unlimited());

// return obj;
// }

// private static JSONArray rotationTargetsToJson(List<RotationTarget> targets) {
// JSONArray arr = new JSONArray();
// targets.forEach((RotationTarget target) -> arr.add(rotationTargetToJson(target)));

// return arr;
// }

// private static JSONObject rotationTargetToJson(RotationTarget target) {
// JSONObject obj = new JSONObject();
// obj.put("waypointRelativePos", target.position());
// obj.put("rotationDegrees", target.rotation().getDegrees());

// return obj;
// }

// private static JSONArray waypointsToJson(List<Waypoint> waypoints) {
// JSONArray arr = new JSONArray();
// waypoints.forEach((Waypoint waypoint) -> arr.add(waypointToJson(waypoint)));

// return arr;
// }

// private static JSONObject waypointToJson(Waypoint waypoint) {
// JSONObject json = new JSONObject();

// json.put("anchor", translation2dToJson(waypoint.anchor()));
// json.put("prevControl", translation2dToJson(waypoint.prevControl()));
// json.put("nextControl", translation2dToJson(waypoint.nextControl()));
// json.put("isLocked", false);
// json.put("linkedName", null);

// return json;
// }

// private static JSONObject translation2dToJson(Translation2d translation2d) {
// JSONObject json = new JSONObject();

// json.put("x", translation2d.getX());
// json.put("y", translation2d.getY());

// return json;
// }

// private static PathPlannerPath generateReefInnerPath(
// Translation2d facePos,
// Rotation2d faceAngle,
// Rotation2d toBranch
// ) {
// Translation2d branchPos = facePos.plus(
// new Translation2d(FieldConstants.ReefConstants.kReefBranchDistMeters / 2, faceAngle.plus(toBranch))
// );

// return generatePathAlongVector(
// branchPos.plus(
// new Translation2d(SwerveConstants.kFrameHeight / 2 + Constants.kBumperWidthMeters, faceAngle)
// ),
// new Translation2d(0.8, faceAngle),
// faceAngle.plus(Rotation2d.k180deg),
// SwerveConstants.kPPReefInnerPathConstraints
// );
// }

// private static PathPlannerPath generateCoralStationInnerPath(CoralStationSide side, CoralStationIndex index) {
// Translation2d stationPos = side == CoralStationSide.C
// ? CoralStationConstants.kBlueCloseCoralStationPose.getTranslation()
// : CoralStationConstants.kBlueFarCoralStationPose.getTranslation();
// Rotation2d faceAngle = CoralStationConstants.getCoralStationFaceAngleBlueAlliance(side);

// Translation2d facePos = stationPos.plus(
// new Translation2d(
// (2 - index.getIndex()) * (CoralStationConstants.kCoralStationOpeningWidthMeters / 3),
// faceAngle.plus(side == CoralStationSide.C ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg)
// )
// );

// return generatePathAlongVector(
// facePos.plus(
// new Translation2d(SwerveConstants.kFrameHeight / 2 + Constants.kBumperWidthMeters, faceAngle)
// ),
// new Translation2d(0.8, faceAngle),
// faceAngle.plus(Rotation2d.k180deg),
// SwerveConstants.kPPCoralStationInnerPathConstraints
// );
// }

// /**
// *
// * @param endPos
// * @param direction
// * @param targetHeading
// * @param constraints
// * @return
// */
// private static PathPlannerPath generatePathAlongVector(
// Translation2d endPos,
// Translation2d direction,
// Rotation2d targetHeading,
// PathConstraints constraints
// ) {
// Translation2d startPos = endPos.plus(direction);
// List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
// new Pose2d(startPos, targetHeading),
// new Pose2d(endPos, targetHeading)
// );

// return new PathPlannerPath(
// waypoints,
// constraints,
// new IdealStartingState(
// constraints.maxVelocity(),
// targetHeading
// ),
// new GoalEndState(0, targetHeading)
// );
// }
// }
