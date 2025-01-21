package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;

public class SuperstructureCommandFactory {
    public static Command autoScore(
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffectorSubsystem,
        ReefBranch branch
    ) {
        return Commands.sequence(
            ElevatorCommandFactory.autoToReefBranch(elevator, branch),
            Commands.waitUntil(() -> {
                Pose2d currentPose = RobotState.getSwervePose();
                ChassisSpeeds currentSpeeds = RobotState.getSwerveCurrentChassisSpeeds();
                double currentSpeed = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
                Pose2d branchPose = branch.getSwerveTargetPoseInner();

                return currentPose.getTranslation().getDistance(
                    branchPose.getTranslation()
                ) < Constants.EndEffectorConstants.kAutoScoreMaxDistMeters &&
                    MathUtil.isNear(
                        currentPose.getRotation().getRadians(),
                        branchPose.getRotation().getRadians(),
                        Constants.EndEffectorConstants.kAutoScoreMaxAngleRadians
                    ) &&
                    currentSpeed < Constants.EndEffectorConstants.kAutoScoreMaxLinearSpeedMetersPerSecond &&
                    currentSpeeds.omegaRadiansPerSecond < Constants.EndEffectorConstants.kAutoScoreMaxAngularSpeedRadiansPerSecond
                    &&
                    elevator.atDesiredPositionMeters();
            }),
            EndEffectorCommandFactory.scoreWithTimeout(endEffectorSubsystem)
        );
    }
}
