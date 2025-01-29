package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoBuilder;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;

public class AutoSelector {
    private static LoggedDashboardChooser<Command> s_autoChooser;
    private static List<LoggedDashboardChooser<String>> s_reefBranches;

    public static void init(SwerveSubsystem swerve, ElevatorSubsystem elevator, EndEffectorSubsystem endEffector) {
        s_autoChooser = new LoggedDashboardChooser<Command>("AutoChooser");
        s_reefBranches = new ArrayList<LoggedDashboardChooser<String>>();
        s_reefBranches.add(new LoggedDashboardChooser<String>("CustomBranch_1"));
        s_reefBranches.add(new LoggedDashboardChooser<String>("CustomBranch_2"));
        s_reefBranches.add(new LoggedDashboardChooser<String>("CustomBranch_3"));

        for (LoggedDashboardChooser<String> customBranchChooser : s_reefBranches) {
            customBranchChooser.addDefaultOption("None", "None");
            for (ReefBranch branch : ReefBranch.kValues) {
                customBranchChooser.addOption(branch.toString(), branch.toString());
            }
        }

        s_autoChooser.addDefaultOption("None", AutoBuilder.none());
        s_autoChooser.addOption("SimpleTaxi", AutoBuilder.simpleTaxiForward(swerve));
        s_autoChooser.addOption("Test", AutoBuilder.testAuto(swerve, elevator, endEffector));
        s_autoChooser.addOption(
            "CustomizableAuto",
            Commands.runOnce(AutoSelector::buildCustomAuto)
        );
    }

    private static Command buildCustomAuto() {
        List<ReefBranch> reefBranches = new ArrayList<>();

        for (LoggedDashboardChooser<String> chooser : s_reefBranches) {
            String name = chooser.get();
            if (name == "None") continue;

            reefBranches.add(
                ReefBranch.fromString(name)
            );
        }
    }

    public static Command get() {
        return s_autoChooser.get();
    }
}
