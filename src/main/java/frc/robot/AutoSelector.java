package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoBuilder;
import frc.robot.commands.ElevatorCommandFactory;
import frc.robot.commands.SwerveCommandFactory;
import frc.robot.commands.WheelRadiusCharacterizationCommand;
import frc.robot.commands.WheelRadiusCharacterizationCommand.Direction;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldConstants.CoralStationConstants.CoralStation;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AutoSelector {
    private static LoggedDashboardChooser<Command> s_autoChooser;
    private static List<LoggedDashboardChooser<String>> s_reefBranches;
    private static List<LoggedDashboardChooser<String>> s_coralStations;

    public static void init(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        s_autoChooser = new LoggedDashboardChooser<>("AutoChooser");
        s_reefBranches = new ArrayList<>();
        s_reefBranches.add(new LoggedDashboardChooser<>("Branch_1"));
        s_reefBranches.add(new LoggedDashboardChooser<>("Branch_3"));
        s_reefBranches.add(new LoggedDashboardChooser<>("Branch_5"));
        for (LoggedDashboardChooser<String> customBranchChooser : s_reefBranches) {
            customBranchChooser.addDefaultOption("None", "None");
            for (ReefBranch branch : ReefBranch.kValues) {
                customBranchChooser.addOption(branch.toString(), branch.toString());
            }
        }

        s_coralStations = new ArrayList<>();
        s_coralStations.add(new LoggedDashboardChooser<>("Coral_2"));
        s_coralStations.add(new LoggedDashboardChooser<>("Coral_4"));
        for (LoggedDashboardChooser<String> customCoral : s_coralStations) {
            customCoral.addDefaultOption("None", "None");
            for (CoralStation station : CoralStation.kValues) {
                customCoral.addOption(station.toString(), station.toString());
            }
        }

        s_autoChooser.addDefaultOption("Comp_None", AutoBuilder.none());
        s_autoChooser.addOption("Comp_SimpleTaxi", AutoBuilder.simpleTaxiForward(swerve));
        s_autoChooser.addOption(
            "Comp_CustomizableAuto",
            Commands.runOnce(() -> buildCustomAuto(swerve, elevator, endEffector, leds).schedule())
        );

        s_autoChooser.addOption("Test", AutoBuilder.testAuto(swerve, elevator, endEffector, leds));
        s_autoChooser.addOption(
            "SysId_WheelRadius",
            new WheelRadiusCharacterizationCommand(swerve, Direction.COUNTER_CLOCKWISE)
        );
        s_autoChooser.addOption("SysId_SwerveDrive", SwerveCommandFactory.makeSysIdDrive(swerve));
        s_autoChooser.addOption("SysId_SwerveTurn", SwerveCommandFactory.makeSysIdTurn(swerve));
        s_autoChooser.addOption("SysId_Elevator", ElevatorCommandFactory.makeSysId(elevator));
    }

    private static Command buildCustomAuto(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        List<ReefBranch> reefBranches = new ArrayList<>();

        for (LoggedDashboardChooser<String> chooser : s_reefBranches) {
            String name = chooser.get();
            Optional<ReefBranch> branch = ReefBranch.fromString(name);
            if (branch.isEmpty()) continue;

            reefBranches.add(branch.get());
        }

        List<CoralStation> coralStations = new ArrayList<>();
        for (LoggedDashboardChooser<String> stationName : s_coralStations) {
            Optional<CoralStation> station = CoralStation.fromString(stationName.get());
            if (station.isEmpty()) continue;

            coralStations.add(station.get());
        }

        Command[] commands = new Command[reefBranches.size() + coralStations.size()];
        int reefIndex = 0;
        int coralIndex = 0;
        for (int i = 0; i < commands.length; i++) {
            if ((i + 1) % 2 == 1) {
                commands[i] = AutoBuilder.makeAutoScoreCommand(
                    reefBranches.get(reefIndex),
                    swerve,
                    elevator,
                    endEffector,
                    leds
                );
                reefIndex++;
                continue;
            }

            commands[i] = AutoBuilder.makeIntakeCommand(coralStations.get(coralIndex), swerve, elevator, endEffector);
            coralIndex++;
        }

        return Commands.sequence(commands);
    }

    public static Command get() {
        return s_autoChooser.get();
    }
}
