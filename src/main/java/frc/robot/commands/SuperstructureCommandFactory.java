package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.util.Conversions;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;
import org.littletonrobotics.junction.Logger;

public class SuperstructureCommandFactory {
    public static Command autoScore(
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffectorSubsystem,
        LEDSubsystem leds,
        ReefBranch branch
    ) {
        return Commands.sequence(
            ElevatorCommandFactory.autoToReefBranch(elevator, branch),
            Commands.waitUntil(() -> {
                // Elevator at position
                boolean elevatorAtDesiredPosition = elevator.atDesiredPosition();

                // Robot is close enough to branch
                Translation2d currentPos = RobotState.getSwervePose().getTranslation();
                Translation2d branchPos = branch.getBranchPosition().toTranslation2d();
                boolean withinRadius = currentPos.getDistance(branchPos)
                    - (Constants.kBumperWidthMeters + Constants.kChassisLengthMeters)
                    <= Conversions.inchesToMeters(3);

                // Robot is moving slow enough
                ChassisSpeeds currentSpeeds = RobotState.getSwerveCurrentChassisSpeeds();
                double currentSpeed = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
                boolean withinSpeedLimit = currentSpeed <= Conversions.inchesToMeters(3);

                boolean shouldScore = elevatorAtDesiredPosition && withinRadius && withinSpeedLimit;
                Logger.recordOutput("SuperstructureAutoScore/ElevatorAtDesiredPosition", elevatorAtDesiredPosition);
                Logger.recordOutput("SuperstructureAutoScore/WithinRadius", withinRadius);
                Logger.recordOutput("SuperstructureAutoScore/WithinSpeedLimit", withinSpeedLimit);
                Logger.recordOutput("SuperstructureAutoScore/ShouldScore", shouldScore);
                return shouldScore;
            }).withTimeout(5),
            EndEffectorCommandFactory.scoreWithTimeout(endEffectorSubsystem),
            LEDCommandFactory.goodThingHappenedCommand(leds)
        );
    }

    public static Command intake(ElevatorSubsystem elevator, EndEffectorSubsystem endEffector) {
        return Commands.parallel(
            ElevatorCommandFactory.toHome(elevator),
            EndEffectorCommandFactory.intake(endEffector)
        );
    }
}
