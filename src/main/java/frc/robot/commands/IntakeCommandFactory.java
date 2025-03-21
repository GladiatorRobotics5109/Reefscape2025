package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.intake.IntakeSubsystem;

public class IntakeCommandFactory {
    public static Command intake(IntakeSubsystem intake) {
        return intake.runOnce(intake::intake);
    }

    public static Command stop(IntakeSubsystem intake) {
        return intake.runOnce(intake::stop);
    }
}
