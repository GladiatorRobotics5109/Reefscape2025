package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.PeriodicUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionMeasurement;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotState {
    private static SwerveSubsystem s_swerve;
    private static VisionSubsystem s_vision;
    private static ElevatorSubsystem s_elevator;

    public static void init(SwerveSubsystem swerve, VisionSubsystem vision, ElevatorSubsystem elevator) {
        s_swerve = swerve;
        s_vision = vision;
        s_elevator = elevator;

        PeriodicUtil.registerPeriodic(RobotState::log);
    }

    public static VisionMeasurement[] getVisionMeasurements() {
        return s_vision.getMeasurements();
    }

    public static Pose2d getSwervePose() {
        return s_swerve.getPose();
    }

    public static SwerveModuleState[] getSwerveModuleStates() {
        return s_swerve.getModuleStates();
    }

    public static double getElevatorCurrentPosition() {
        return s_elevator.getCurrentPosition();
    }

    public static double getElevatorDesiredPosition() {
        return s_elevator.getDesiredPosition();
    }

    public static void log() {
        Logger.recordOutput(SwerveConstants.kLogPath + "/currentPose", getSwervePose());
        Logger.recordOutput(SwerveConstants.kLogPath + "/currentModuleStates", getSwerveModuleStates());

        Logger.recordOutput(SwerveConstants.kLogPath + "/currentPosition", getElevatorCurrentPosition());
        Logger.recordOutput(SwerveConstants.kLogPath + "/desiredPosition", getElevatorDesiredPosition());
    }
}
