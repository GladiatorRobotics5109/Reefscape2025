package frc.robot;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.PeriodicUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionMeasurement;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;
import org.littletonrobotics.junction.Logger;

import java.util.HashSet;
import java.util.Set;

public class RobotState {
    private static SwerveSubsystem s_swerve;
    private static VisionSubsystem s_vision;
    private static ElevatorSubsystem s_elevator;
    private static EndEffectorSubsystem s_endEffector;

    private static Set<ReefBranch> s_scoredBranches;

    public static void init(
        SwerveSubsystem swerve,
        VisionSubsystem vision,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector
    ) {
        s_swerve = swerve;
        s_vision = vision;
        s_elevator = elevator;
        s_endEffector = endEffector;

        s_scoredBranches = new HashSet<>();

        PeriodicUtil.registerPeriodic(RobotState::log);
    }

    public static VisionMeasurement[] getVisionMeasurements() { return s_vision.getMeasurements(); }

    public static Pose2d getSwervePose() { return s_swerve.getPose(); }

    public static SwerveModuleState[] getSwerveModuleStates() { return s_swerve.getModuleStates(); }

    public static ChassisSpeeds getSwerveCurrentChassisSpeeds() { return s_swerve.getCurrentChassisSpeeds(); }

    public static double getElevatorCurrentPositionMeters() { return s_elevator.getCurrentPositionElevator(); }

    public static double getElevatorDesiredPositionMeters() { return s_elevator.getDesiredPositionElevator(); }

    public static boolean getElevatorAtDesiredPosition() { return s_elevator.atDesiredPosition(); }

    public static boolean getEndEffectorHasCoral() { return s_endEffector.hasCoral(); }

    public static boolean getEndEffectorHasLeadingEdgeCoral() { return s_endEffector.hasLeadingEdgeCoral(); }

    public static void addScoredBranch(ReefBranch branch) {
        s_scoredBranches.add(branch);
    }

    public static boolean hasScoredOnBranch(ReefBranch branch) { return s_scoredBranches.contains(branch); }

    public static Set<ReefBranch> getScoredBranches() { return s_scoredBranches; }

    public static void log() {
        Logger.recordOutput(SwerveConstants.kLogPath + "/CurrentPose", getSwervePose());
        Logger.recordOutput(SwerveConstants.kLogPath + "/CurrentModuleStates", getSwerveModuleStates());

        Logger.recordOutput(ElevatorConstants.kLogPath + "/CurrentPositionMeters", getElevatorCurrentPositionMeters());
        Logger.recordOutput(ElevatorConstants.kLogPath + "/DesiredPositionMeters", getElevatorDesiredPositionMeters());
        Logger.recordOutput(ElevatorConstants.kLogPath + "/AtDesiredPosition", getElevatorAtDesiredPosition());

        Logger.recordOutput(EndEffectorConstants.kLogPath + "/HasCoral", getEndEffectorHasCoral());
        Logger.recordOutput(
            EndEffectorConstants.kLogPath + "/HasLeadingEdgeCoral",
            getEndEffectorHasLeadingEdgeCoral()
        );

        Logger.recordOutput(
            "ScoredBranches",
            s_scoredBranches.stream().map(ReefBranch::getBranchPosition).toArray(Translation3d[]::new)
        );
    }
}
