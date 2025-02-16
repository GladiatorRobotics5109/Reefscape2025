package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;

public class EndEffectorCommandFactory {
    public static Command score(EndEffectorSubsystem endEffector) {
        return Commands.sequence(
            endEffector.runOnce(endEffector::setScore),
            Commands.waitUntil(() -> !endEffector.hasCoral())
        ).finallyDo(endEffector::stop);
    }

    public static Command scoreWithTimeout(EndEffectorSubsystem endEffector) {
        return score(endEffector).withTimeout(EndEffectorConstants.kScoreTimeoutSeconds);
    }

    // TOOD: implement this
    public static Command intake(EndEffectorSubsystem endEffector) {
        return Commands.none();
    }
}
