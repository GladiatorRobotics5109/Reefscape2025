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
        return Commands.sequence(
            endEffector.runOnce(endEffector::setScore),
            Commands.waitUntil(() -> !endEffector.hasCoral())
        ).finallyDo(endEffector::stop).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public static Command scoreWithTimeout(EndEffectorSubsystem endEffector) {
        return score(endEffector).withTimeout(EndEffectorConstants.kScoreTimeoutSeconds);
    }

    public static Command intake(EndEffectorSubsystem endEffector) {
        return Commands.sequence(
            Commands.runOnce(endEffector::setIntake, endEffector),
            Commands.either(
                // If sim, don't wait for coral bc coral sensor is not simulated
                Commands.waitSeconds(1),
                Commands.waitUntil(endEffector::hasLeadingEdgeCoral),
                Util::isSim
            ),
            Commands.runOnce(endEffector::setIntakeSlow, endEffector),
            Commands.either(
                Commands.waitSeconds(1),
                Commands.waitUntil(() -> !endEffector.hasLeadingEdgeCoral()),
                Util::isSim
            ),
            Commands.runOnce(endEffector::stop, endEffector)
        );
    }
}
