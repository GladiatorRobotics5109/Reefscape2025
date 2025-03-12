package frc.robot.subsystems.swerve;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedgyro.LoggedGyro;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedgyro.LoggedGyroIO;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedgyro.LoggedGyroIOPigeon;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedgyro.LoggedGyroIOSim;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.swervemodule.SwerveModule;
import frc.robot.subsystems.swerve.swervemodule.SwerveModuleIO;
import frc.robot.subsystems.swerve.swervemodule.SwerveModuleIOSimTalonFx;
import frc.robot.subsystems.swerve.swervemodule.SwerveModuleIOTalonFx;
import frc.robot.subsystems.vision.VisionMeasurement;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule m_moduleFL;
    private final SwerveModule m_moduleFR;
    private final SwerveModule m_moduleBL;
    private final SwerveModule m_moduleBR;

    private final LoggedGyro m_gyro;

    private final SwerveDriveKinematics m_kinematics;
    private final SwerveDrivePoseEstimator m_poseEstimator;

    public SwerveSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_moduleFL = new SwerveModule(
                    0,
                    new SwerveModuleIOTalonFx(
                        SwerveConstants.SwerveModuleConstants.kFrontLeftDrivePort,
                        SwerveConstants.SwerveModuleConstants.kFrontLeftTurnPort,
                        SwerveConstants.SwerveModuleConstants.kFrontLeftEncoderPort,
                        SwerveConstants.SwerveModuleConstants.kUseFOC
                    ),
                    SwerveConstants.SwerveModuleConstants.kUseMotorPID
                );
                m_moduleFR = new SwerveModule(
                    1,
                    new SwerveModuleIOTalonFx(
                        SwerveConstants.SwerveModuleConstants.kFrontRightDrivePort,
                        SwerveConstants.SwerveModuleConstants.kFrontRightTurnPort,
                        SwerveConstants.SwerveModuleConstants.kFrontRightEncoderPort,
                        SwerveConstants.SwerveModuleConstants.kUseFOC
                    ),
                    SwerveConstants.SwerveModuleConstants.kUseMotorPID
                );
                m_moduleBL = new SwerveModule(
                    2,
                    new SwerveModuleIOTalonFx(
                        SwerveConstants.SwerveModuleConstants.kBackLeftDrivePort,
                        SwerveConstants.SwerveModuleConstants.kBackLeftTurnPort,
                        SwerveConstants.SwerveModuleConstants.kBackLeftEncoderPort,
                        SwerveConstants.SwerveModuleConstants.kUseFOC
                    ),
                    SwerveConstants.SwerveModuleConstants.kUseMotorPID
                );
                m_moduleBR = new SwerveModule(
                    3,
                    new SwerveModuleIOTalonFx(
                        SwerveConstants.SwerveModuleConstants.kBackRightDrivePort,
                        SwerveConstants.SwerveModuleConstants.kBackRightTurnPort,
                        SwerveConstants.SwerveModuleConstants.kBackRightEncoderPort,
                        SwerveConstants.SwerveModuleConstants.kUseFOC
                    ),
                    SwerveConstants.SwerveModuleConstants.kUseMotorPID
                );

                m_gyro = new LoggedGyro(
                    "Subsystems/Swerve/Gyro",
                    new LoggedGyroIOPigeon(SwerveConstants.kPigeonPort, "drivetrain")
                );

                break;
            case SIM:
                m_moduleFL = new SwerveModule(
                    0,
                    new SwerveModuleIOSimTalonFx(
                        SwerveConstants.SwerveModuleConstants.kFrontLeftDrivePort,
                        SwerveConstants.SwerveModuleConstants.kFrontLeftTurnPort,
                        SwerveConstants.SwerveModuleConstants.kUseFOC
                    ),
                    SwerveConstants.SwerveModuleConstants.kUseMotorPID
                );
                m_moduleFR = new SwerveModule(
                    1,
                    new SwerveModuleIOSimTalonFx(
                        SwerveConstants.SwerveModuleConstants.kFrontRightDrivePort,
                        SwerveConstants.SwerveModuleConstants.kFrontRightTurnPort,
                        SwerveConstants.SwerveModuleConstants.kUseFOC
                    ),
                    SwerveConstants.SwerveModuleConstants.kUseMotorPID
                );
                m_moduleBL = new SwerveModule(
                    2,
                    new SwerveModuleIOSimTalonFx(
                        SwerveConstants.SwerveModuleConstants.kBackLeftDrivePort,
                        SwerveConstants.SwerveModuleConstants.kBackLeftTurnPort,
                        SwerveConstants.SwerveModuleConstants.kUseFOC
                    ),
                    SwerveConstants.SwerveModuleConstants.kUseMotorPID
                );
                m_moduleBR = new SwerveModule(
                    3,
                    new SwerveModuleIOSimTalonFx(
                        SwerveConstants.SwerveModuleConstants.kBackRightDrivePort,
                        SwerveConstants.SwerveModuleConstants.kBackRightTurnPort,
                        SwerveConstants.SwerveModuleConstants.kUseFOC
                    ),
                    SwerveConstants.SwerveModuleConstants.kUseMotorPID
                );
                // m_moduleFL = new SwerveModule(0, new SwerveModuleIOSim(), false);
                // m_moduleFR = new SwerveModule(1, new SwerveModuleIOSim(), false);
                // m_moduleBL = new SwerveModule(2, new SwerveModuleIOSim(), false);
                // m_moduleBR = new SwerveModule(3, new SwerveModuleIOSim(), false);

                m_gyro = new LoggedGyro("Subsystems/Swerve/Gyro", new LoggedGyroIOSim());

                break;
            default:
                m_moduleFL = new SwerveModule(0, new SwerveModuleIO() {}, false);
                m_moduleFR = new SwerveModule(1, new SwerveModuleIO() {}, false);
                m_moduleBL = new SwerveModule(2, new SwerveModuleIO() {}, false);
                m_moduleBR = new SwerveModule(3, new SwerveModuleIO() {}, false);

                m_gyro = new LoggedGyro("Subsystems/Swerve/Gyro", new LoggedGyroIO() {});

                break;
        }

        m_kinematics = new SwerveDriveKinematics(
            SwerveConstants.SwerveModuleConstants.kModulePosFL,
            SwerveConstants.SwerveModuleConstants.kModulePosFR,
            SwerveConstants.SwerveModuleConstants.kModulePosBL,
            SwerveConstants.SwerveModuleConstants.kModulePosBR
        );

        m_gyro.resetYaw();
        m_gyro.setYaw(Rotation2d.fromDegrees(180));

        m_poseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics,
            m_gyro.getYaw(),
            getModulePositions(),
            SwerveConstants.kStartingPose
        );

        m_poseEstimator.setVisionMeasurementStdDevs(SwerveConstants.kVisionStdDevs);

        AutoBuilder.configure(
            this::getPose,
            m_poseEstimator::resetPose,
            this::getCurrentChassisSpeeds,
            (speeds, feedForward) -> drive(speeds, false),
            new PPHolonomicDriveController(SwerveConstants.kPPTranslationPID, SwerveConstants.kPPRotaitonPID),
            SwerveConstants.kPPConfig,
            frc.robot.commands.AutoBuilder::shouldFlip,
            this
        );

        PathPlannerLogging.setLogActivePathCallback(
            (List<Pose2d> path) -> Logger.recordOutput(
                SwerveConstants.kLogPath + "/PathPlanner/ActivePath",
                path.toArray(new Pose2d[0])
            )
        );
        PathPlannerLogging.setLogCurrentPoseCallback(
            (Pose2d pose) -> Logger.recordOutput(SwerveConstants.kLogPath + "/PathPlanner/CurrentPose", pose)
        );
        PathPlannerLogging.setLogTargetPoseCallback(
            (Pose2d pose) -> Logger.recordOutput(SwerveConstants.kLogPath + "/PathPlanner/DesiredPose", pose)
        );
    }

    /**
     *
     * @param vx velocity in m/s
     * @param vy velocity in m/s
     * @param vrot rotational velocity in rad/s
     * @param fieldRelative
     */
    public void drive(double vx, double vy, double vrot, boolean fieldRelative) {
        Rotation2d headingOffset = Util.getAlliance() == Alliance.Red
            ? Rotation2d.fromDegrees(180)
            : Rotation2d.fromDegrees(0);
        ChassisSpeeds desiredSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vrot, getHeading().plus(headingOffset))
            : new ChassisSpeeds(vx, vy, vrot);
        desiredSpeeds = ChassisSpeeds.discretize(desiredSpeeds, Constants.kLoopPeriodSecs);

        SwerveModuleState[] desiredStates = m_kinematics.toSwerveModuleStates(desiredSpeeds);
        SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
        optimizedStates[0] = m_moduleFL.setDesiredState(desiredStates[0]);
        optimizedStates[1] = m_moduleFR.setDesiredState(desiredStates[1]);
        optimizedStates[2] = m_moduleBL.setDesiredState(desiredStates[2]);
        optimizedStates[3] = m_moduleBR.setDesiredState(desiredStates[3]);

        Logger.recordOutput(SwerveConstants.kLogPath + "/desiredSpeeds", desiredSpeeds);
        Logger.recordOutput(SwerveConstants.kLogPath + "/desiredModuleStates", optimizedStates);
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative);
    }

    public void stopAndX() {
        SwerveModuleState flBr = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
        SwerveModuleState frBL = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45));

        m_moduleFL.setDesiredState(flBr, false);
        m_moduleFR.setDesiredState(frBL, false);
        m_moduleBL.setDesiredState(frBL, false);
        m_moduleBR.setDesiredState(flBr, false);

        Logger.recordOutput(
            SwerveConstants.kLogPath
                + "/desiredModuleStates",
            new SwerveModuleState[] {
                flBr,
                frBL,
                frBL,
                flBr
            }
        );
    }

    public Pose2d getPose() { return m_poseEstimator.getEstimatedPosition(); }

    public Rotation2d getHeading() { return getPose().getRotation(); }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_moduleFL.getPosition(),
            m_moduleFR.getPosition(),
            m_moduleBL.getPosition(),
            m_moduleBR.getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_moduleFL.getState(),
            m_moduleFR.getState(),
            m_moduleBL.getState(),
            m_moduleBR.getState()
        };
    }

    public double[] getModuleTurnVelocities() {
        return new double[] {
            m_moduleFL.getTurnVelocityRadPerSec(),
            m_moduleFR.getTurnVelocityRadPerSec(),
            m_moduleBL.getTurnVelocityRadPerSec(),
            m_moduleBR.getTurnVelocityRadPerSec(),
        };
    }

    /**
     * Should be used for debug/testing purposes only
     *
     * @return
     */
    public SwerveModule[] getSwerveModules() {
        return new SwerveModule[] {
            m_moduleFL,
            m_moduleFR,
            m_moduleBL,
            m_moduleBR
        };
    }

    /** Get the position of all drive wheels in radians. */
    public double[] getWheelRadiusCharacterizationPosition() {
        return new double[] {
            m_moduleFL.getDrivePositionRads(),
            m_moduleFR.getDrivePositionRads(),
            m_moduleBL.getDrivePositionRads(),
            m_moduleBR.getDrivePositionRads()
        };
    }

    public ChassisSpeeds getCurrentChassisSpeeds() { return m_kinematics.toChassisSpeeds(getModuleStates()); }

    public void setPosition(Pose2d pose) {
        m_poseEstimator.resetPosition(m_gyro.getYaw(), getModulePositions(), pose);
    }

    public void addVisionMeasurements(VisionMeasurement... measurements) {
        for (VisionMeasurement measurement : measurements) {
            Logger.recordOutput(
                SwerveConstants.kLogPath + "/VisionMeasurements/" + measurement.cameraName(),
                measurement
            );

            m_poseEstimator.addVisionMeasurement(measurement.estimatedPose(), measurement.timestamp());
        }
    }

    public void updatePose() {
        m_poseEstimator.update(m_gyro.getYaw(), getModulePositions());
    }

    @Override
    public void periodic() {
        m_moduleFL.periodic();
        m_moduleFR.periodic();
        m_moduleBL.periodic();
        m_moduleBR.periodic();

        if (m_gyro.isSim()) {
            double vrot = m_kinematics.toChassisSpeeds(
                m_moduleFL.getState(),
                m_moduleFR.getState(),
                m_moduleBL.getState(),
                m_moduleBR.getState()
            ).omegaRadiansPerSecond;
            double newRot = m_gyro.getYaw().getRadians() + (vrot * Constants.kLoopPeriodSecs);

            m_gyro.setYaw(Rotation2d.fromRadians(newRot));
        }

        updatePose();
    }
}
