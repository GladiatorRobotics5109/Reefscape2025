package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ClimbCommandFactory {
    public static Command setVoltage(ClimbSubsystem climb, double volts) {
        return climb.runOnce(() -> climb.setVoltage(volts));
    }

    public static Command setDesiredPosition(ClimbSubsystem climb, Rotation2d position) {
        return climb.runOnce(() -> climb.setDesiredPosition(position));
    }

    public static Command climb(ClimbSubsystem climb) {
        return Commands.sequence(
            setVoltage(climb, ClimbConstants.kClimbingVoltage),
            Commands.waitUntil(climb::atClimbPosition),
            setVoltage(climb, ClimbConstants.kHoldingVoltage)
        );
    }

    public static Command prepareClimb(ClimbSubsystem climb) {
        return setDesiredPosition(climb, ClimbConstants.kCagePosition);
    }
}
