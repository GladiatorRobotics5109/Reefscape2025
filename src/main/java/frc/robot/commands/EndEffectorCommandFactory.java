package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.util.Util;

public class EndEffectorCommandFactory {
    public static Command setVoltage(EndEffectorSubsystem endEffector, double volts) {
        return endEffector.runOnce(() -> endEffector.setVoltage(volts));
    }

    public static Command score(EndEffectorSubsystem endEffector) {
        if (Util.isSim()) {
            return Commands.sequence(endEffector.runOnce(endEffector::setScore), Commands.waitSeconds(2));
        }

        return Commands.sequence(
            endEffector.runOnce(endEffector::setScore),
            Commands.waitUntil(() -> !endEffector.hasCoral())
        ).finallyDo(endEffector::stop).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public static Command scoreWithTimeout(EndEffectorSubsystem endEffector) {
        return score(endEffector).withTimeout(EndEffectorConstants.kScoreTimeoutSeconds);
    }

    public static Command intake(EndEffectorSubsystem endEffector) {
        if (Util.isSim()) {
            return Commands.waitSeconds(2.0);
        }

        return Commands.sequence(
            Commands.runOnce(endEffector::setIntake, endEffector),
            Commands.waitUntil(endEffector::hasLeadingEdgeCoral),
            Commands.waitSeconds(0.08),
            Commands.runOnce(endEffector::setIntakeSlow, endEffector),
            Commands.waitUntil(() -> !endEffector.hasLeadingEdgeCoral()),
            Commands.runOnce(
                () -> endEffector.setVoltage(EndEffectorConstants.kIntakeSlowSlowVoltage),
                endEffector
            ),
            Commands.waitUntil(endEffector::hasLeadingEdgeCoral),
            Commands.waitSeconds(0.150),
            Commands.runOnce(endEffector::stop, endEffector)
        );
    }
}
