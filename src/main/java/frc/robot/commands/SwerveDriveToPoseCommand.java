package frc.robot.commands;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.math.controller.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class SwerveDriveToPoseCommand extends Command {
    private static final String kLogPath = "SwerveDriveToPoseCommand";

    private final SwerveSubsystem m_swerve;

    private final Pose2d m_desiredPose;

    private final PIDController m_xPID;
    private final PIDController m_yPID;
    private final PIDController m_rotPID;

    private Pose2d m_currentPose;
    private ChassisSpeeds m_currentSpeeds;

    private final Debouncer m_atTranslationDebounce;
    private final Debouncer m_atRotationDebounce;

    private boolean m_atTranslation;
    private boolean m_atRotation;
    private boolean m_atTranslationVel;
    private boolean m_atRotationVel;

    public SwerveDriveToPoseCommand(
        SwerveSubsystem swerve,
        Pose2d desiredPose,
        PIDConstants translationPID,
        PIDConstants rotationPID
    ) {
        m_swerve = swerve;
        m_desiredPose = desiredPose;
        m_xPID = translationPID.getPIDController();
        m_yPID = translationPID.getPIDController();
        m_rotPID = rotationPID.getPIDController();

        m_currentPose = m_swerve.getPose();
        m_currentSpeeds = m_swerve.getCurrentChassisSpeeds();

        m_atTranslation = false;
        m_atRotation = false;

        m_atTranslationDebounce = new Debouncer(SwerveConstants.kDriveToPoseTranslationDebounce);
        m_atRotationDebounce = new Debouncer(SwerveConstants.kDriveToPoseRotationDebounce);

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        Logger.recordOutput(kLogPath + "/DesiredPose", m_desiredPose);
        Logger.recordOutput(kLogPath + "/CurrentPose", m_currentPose);
        Logger.recordOutput(kLogPath + "/ErrorX", 0.0);
        Logger.recordOutput(kLogPath + "/ErrorY", 0.0);
        Logger.recordOutput(kLogPath + "/ErrorRot", 0.0);
        Logger.recordOutput(kLogPath + "/VelX", 0.0);
        Logger.recordOutput(kLogPath + "/VelY", 0.0);
        Logger.recordOutput(kLogPath + "/VelRot", 0.0);
        Logger.recordOutput(kLogPath + "/AtTranslation", m_atTranslation);
        Logger.recordOutput(kLogPath + "/AtRotation", m_atRotation);
        Logger.recordOutput(kLogPath + "/AtTranslationVel", m_atTranslationVel);
        Logger.recordOutput(kLogPath + "/AtRotationVel", m_atRotationVel);
    }

    @Override
    public void execute() {
        m_currentPose = m_swerve.getPose();
        m_currentSpeeds = m_swerve.getCurrentChassisSpeeds();

        double xVel = MathUtil.clamp(
            m_xPID.calculate(m_currentPose.getX(), m_desiredPose.getX()),
            -SwerveConstants.kDriveToPoseMaxSpeedMetersPerSec,
            SwerveConstants.kDriveToPoseMaxSpeedMetersPerSec
        );
        double yVel = MathUtil.clamp(
            m_yPID.calculate(m_currentPose.getY(), m_desiredPose.getY()),
            -SwerveConstants.kDriveToPoseMaxSpeedMetersPerSec,
            SwerveConstants.kDriveToPoseMaxSpeedMetersPerSec
        );
        double rotVel = MathUtil.clamp(
            m_rotPID.calculate(m_currentPose.getRotation().getRadians(), m_desiredPose.getRotation().getRadians()),
            -SwerveConstants.kDriveToPoseMaxRotationSpeedRadPerSec,
            SwerveConstants.kDriveToPoseMaxRotationSpeedRadPerSec
        );

        if (Util.getAlliance() == Alliance.Red) {
            xVel = -xVel;
            yVel = -yVel;
        }

        // Rotation2d headingOffset = Util.getAlliance() == Alliance.Red
        //     ? Rotation2d.fromDegrees(180)
        //     : Rotation2d.fromDegrees(0);

        // ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
        //     xVel,
        //     yVel,
        //     rotVel,
        //     m_swerve.getHeading().plus(headingOffset)
        // );
        // robotRelative.vyMetersPerSecond *= SwerveConstants.kDriveToPoseYScale;
        // m_swerve.drive(robotRelative, false);

        m_swerve.drive(xVel, yVel, rotVel, true);

        m_atTranslation = m_currentPose.getTranslation().getDistance(m_desiredPose.getTranslation())
            <= SwerveConstants.kDriveToPoseTranslationToleranceMeters;
        m_atRotation = m_currentPose.getRotation().getRadians() - m_desiredPose.getRotation().getRadians()
            <= SwerveConstants.kDriveToPoseRotationToleranceRad;

        m_atTranslationVel = Math.hypot(m_currentSpeeds.vxMetersPerSecond, m_currentSpeeds.vyMetersPerSecond)
            <= SwerveConstants.kDriveToPoseTranslationVelocityToleranceMetersPerSec;
        m_atRotationVel = m_currentSpeeds.omegaRadiansPerSecond
            <= SwerveConstants.kDriveToPoseRotationVelocityToleranceRadPerSec;

        Logger.recordOutput(kLogPath + "/CurrentPose", m_currentPose);
        Logger.recordOutput(kLogPath + "/ErrorX", m_xPID.getError());
        Logger.recordOutput(kLogPath + "/ErrorY", m_yPID.getError());
        Logger.recordOutput(kLogPath + "/ErrorRot", m_rotPID.getError());
        Logger.recordOutput(kLogPath + "/VelX", xVel);
        Logger.recordOutput(kLogPath + "/VelY", yVel);
        Logger.recordOutput(kLogPath + "/VelRot", rotVel);
        Logger.recordOutput(kLogPath + "/AtTranslation", m_atTranslation);
        Logger.recordOutput(kLogPath + "/AtRotation", m_atRotation);
        Logger.recordOutput(kLogPath + "/AtTranslationVel", m_atTranslationVel);
        Logger.recordOutput(kLogPath + "/AtRotationVel", m_atRotationVel);
    }

    @Override
    public boolean isFinished() {
        return m_atTranslationDebounce.calculate(m_atTranslation)
            && m_atRotationDebounce.calculate(m_atRotation)
            && m_atTranslationVel
            && m_atRotationVel;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(0.0, 0.0, 0.0, true);
    }
}
