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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AutoChooser {
    private static LoggedDashboardChooser<Command> s_autoChooser;
    private static LoggedNetworkNumber s_preAutoDelay;
    private static List<LoggedDashboardChooser<String>> s_reefBranches;
    private static List<LoggedDashboardChooser<String>> s_coralStations;

    private static SwerveSubsystem s_swerve;
    private static ElevatorSubsystem s_elevator;
    private static EndEffectorSubsystem s_endEffector;
    private static LEDSubsystem s_leds;

    public static void init(
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        s_swerve = swerve;
        s_elevator = elevator;
        s_endEffector = endEffector;
        s_leds = leds;

        s_autoChooser = new LoggedDashboardChooser<>("AutoChooser");
        s_preAutoDelay = new LoggedNetworkNumber("PreAutoDelay", 0.0);
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

        s_autoChooser.addDefaultOption("Comp_None", AutoBuilder.none(swerve));
        s_autoChooser.addOption("Comp_SimpleTaxi", AutoBuilder.simpleTaxiForward(swerve));
        s_autoChooser.addOption(
            "Comp_CustomizableAuto",
            Commands.runOnce(() -> buildCustomAuto(swerve, elevator, endEffector, leds).schedule())
        );
        s_autoChooser.addOption("Comp_PP-B_6_L1G2", AutoBuilder.auto_PP_B6_L1G2(swerve, elevator, endEffector, leds));
        s_autoChooser.addOption("Comp_PP-B_6_L2G2", AutoBuilder.auto_PP_B6_L2G2(swerve, elevator, endEffector, leds));
        s_autoChooser.addOption(
            "Comp_PP-B_6-R_L4G2-Leave",
            AutoBuilder.auto_PP_B6_L4G2_Leave(swerve, elevator, endEffector, leds)
        );
        s_autoChooser.addOption(
            "Comp_PP-B_6-R_L4G2-C_F3-R_L4G1-Leave",
            AutoBuilder.auto_PP_B6_L4G2_F3_L4G1_Leave(swerve, elevator, endEffector, leds)
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

    private static Command autoPrefix() {
        return Commands.waitSeconds(s_preAutoDelay.get());
    }

    public static Command get() {
        return autoPrefix().andThen(s_autoChooser.get());
    }
}
