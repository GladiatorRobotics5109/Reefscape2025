package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.PeriodicUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ElevatorConstants;
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

    public static VisionMeasurement[] getVisionMeasurements() { return s_vision.getMeasurements(); }

    public static Pose2d getSwervePose() { return s_swerve.getPose(); }

    public static SwerveModuleState[] getSwerveModuleStates() { return s_swerve.getModuleStates(); }

    public static ChassisSpeeds getSwerveCurrentChassisSpeeds() { return s_swerve.getCurrentChassisSpeeds(); }

    public static double getElevatorCurrentPositionMeters() { return s_elevator.getCurrentPositionElevatorRelative(); }

    public static double getElevatorDesiredPositionMeters() { return s_elevator.getDesiredPositionElevatorRelative(); }

    public static boolean getElevatorAtDesiredPosition() { return s_elevator.atDesiredPosition(); }

    public static void log() {
        Logger.recordOutput(SwerveConstants.kLogPath + "/currentPose", getSwervePose());
        Logger.recordOutput(SwerveConstants.kLogPath + "/currentModuleStates", getSwerveModuleStates());

        Logger.recordOutput(ElevatorConstants.kLogPath + "/currentPositionMeters", getElevatorCurrentPositionMeters());
        Logger.recordOutput(ElevatorConstants.kLogPath + "/desiredPositionMeters", getElevatorDesiredPositionMeters());
    }

    public static record SwerveSetpoint(Pose2d position, Rotation2d heading) {}

    public static record ElevatorSetpoint(double positionMeters) {}

    public static record RobotSetpoint(SwerveSetpoint swerveSetpoint, ElevatorSetpoint elevatorSetpoint) {}
}
