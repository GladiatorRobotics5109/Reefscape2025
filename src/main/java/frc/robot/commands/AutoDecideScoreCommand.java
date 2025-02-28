package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefConstants.ReefHeight;

public class AutoDecideScoreCommand extends Command {
    private Command m_scoreCommand;
    private final ReefHeight m_height;

    private final SwerveSubsystem m_swerve;
    private final ElevatorSubsystem m_elevator;
    private final EndEffectorSubsystem m_endEffector;
    private final LEDSubsystem m_leds;

    public AutoDecideScoreCommand(
        ReefHeight height,
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        m_height = height;
        m_swerve = swerve;
        m_elevator = elevator;
        m_endEffector = endEffector;
        m_leds = leds;

        setName("AutoDecideScoreCommand");
        addRequirements(m_swerve, m_elevator, m_endEffector, m_leds);
    }

    @Override
    public void initialize() {
        m_scoreCommand = AutoBuilder.makeAutoScoreCommand(
            findClosestBranchAtHeight(m_height),
            m_swerve,
            m_elevator,
            m_endEffector,
            m_leds
        );

        m_scoreCommand.initialize();
    };

    @Override
    public void execute() {
        if (m_scoreCommand == null) return;

        m_scoreCommand.execute();
    }

    @Override
    public boolean isFinished() {
        if (m_scoreCommand == null) return false;

        return m_scoreCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (m_scoreCommand == null) return;

        m_scoreCommand.end(interrupted);
    }

    private static FieldConstants.ReefConstants.ReefBranch findClosestBranchAtHeight(ReefHeight height) {
        Pose2d pose = RobotState.getSwervePose();

        FieldConstants.ReefConstants.ReefBranch closest = new FieldConstants.ReefConstants.ReefBranch(
            height,
            FieldConstants.ReefConstants.ReefFace.E,
            FieldConstants.ReefConstants.ReefIndex.One
        );
        double closestDistMeters = -1;

        for (FieldConstants.ReefConstants.ReefBranch branch : FieldConstants.ReefConstants.ReefBranch.kValues) {
            if (branch.getHeight() != height) continue;
            if (RobotState.hasScoredOnBranch(branch)) continue;

            double dist = branch.getBranchPosition().toTranslation2d().getDistance(pose.getTranslation());
            if (dist < closestDistMeters || closestDistMeters == -1.0) {
                closestDistMeters = dist;
                closest = branch;
            }
        }

        return closest;
    }
}
