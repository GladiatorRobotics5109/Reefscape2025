package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.algae.AlgaeSubsystem;

public class AlgaeCommandFactory {
    public static Command toRemove(AlgaeSubsystem algae) {
        return algae.runOnce(algae::toRemove);
    }

    public static Command toStow(AlgaeSubsystem algae) {
        return algae.runOnce(algae::toStow);
    }
}
