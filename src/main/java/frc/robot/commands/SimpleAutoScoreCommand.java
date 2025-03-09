package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;
import frc.robot.util.FieldConstants.ReefConstants.ReefHeight;
import org.littletonrobotics.junction.Logger;

public class SimpleAutoScoreCommand extends Command {
    private final ReefBranch m_branch; 
    
    private final SwerveSubsystem m_swerve;
    private final ElevatorSubsystem m_elevator;
    private final EndEffectorSubsystem m_endEffector;
    private final LEDSubsystem m_leds;
    
    private final Pose2d m_targetPose; 
    private final Pose2d m_startingPose;
    private Pose2d m_desiredPose;
    private final double m_rate;
    private final Timer m_timer;
    
    private final PIDController m_xPID;
    private final PIDController m_yPID;
    private final PIDController m_rotPID;
    
    public SimpleAutoScoreCommand(
        ReefBranch branch,
        SwerveSubsystem swerve,
        ElevatorSubsystem elevator,
        EndEffectorSubsystem endEffector,
        LEDSubsystem leds
    ) {
        m_branch = branch;
        
        m_swerve = swerve;
        m_elevator = elevator;
        m_endEffector = endEffector;
        m_leds = leds;
        
        m_xPID = SwerveConstants.kSimpleAutoScoreXPID.getPIDController();
        m_yPID = SwerveConstants.kSimpleAutoScoreYPID.getPIDController();
        m_rotPID = SwerveConstants.kSimpleAutoScoreRotPID.getPIDController();
        m_rate = SwerveConstants.kSimpleAutoScoreRate;
        m_timer = new Timer();
        
        m_targetPose = m_branch.getSwerveTargetPoseInner();
        Logger.recordOutput("SimpleAutoScoreCommand/TargetPose", m_targetPose);
        m_startingPose = m_branch.getSwerveTargetPoseOuter();
        Logger.recordOutput("SimpleAutoScoreCommand/StartingPose", m_startingPose);
        m_desiredPose = m_startingPose;
        
        setName("SimpleAutoScoreCommand");
        addRequirements(m_swerve, m_elevator, m_endEffector, m_leds);
    }
    
    @Override
    public void initialize() {
        m_timer.restart();
    };
    
    @Override
    public void execute() {
        m_desiredPose = m_startingPose.interpolate(m_targetPose, m_timer.get() * m_rate);
        Pose2d currentPose = m_swerve.getPose();
        double desiredX = m_desiredPose.getX();
        double desiredY = m_desiredPose.getY();
        double desiredRot = m_desiredPose.getRotation().getRadians();
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
            m_xPID.calculate(currentPose.getX(), desiredX),
            m_yPID.calculate(currentPose.getY(), desiredY),
            m_rotPID.calculate(currentPose.getRotation().getRadians(), desiredRot)
        );
        
        Logger.recordOutput("SimpleAutoScoreCommand/DesiredPose", m_desiredPose);
        Logger.recordOutput("SimpleAutoScoreCommand/CurrentPose", currentPose);
        Logger.recordOutput("SimpleAutoScoreCommand/DesiredSpeeds", desiredSpeeds);
        m_swerve.drive(desiredSpeeds, true);
        
        
    }
    
    @Override
    public boolean isFinished() {
    }
    
    @Override
    public void end(boolean interrupted) {
    }
}
