package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionMeasurement;
import frc.robot.subsystems.vision.VisionSubsystem;

public class StateMachine {
    private static SwerveSubsystem s_swerve;
    private static VisionSubsystem s_vision;
    private static ElevatorSubsystem s_elevator;

    public static void init(SwerveSubsystem swerve, VisionSubsystem vision, ElevatorSubsystem elevator) {
        s_swerve = swerve;
        s_vision = vision;
        s_elevator = elevator;
    }

    public static VisionMeasurement[] getVisionMeasurements() {
        return s_vision.getMeasurements();
    }

    public static Pose2d getPose() {
        return s_swerve.getPose();
    }

    public static double getElevatorPosition() {
        return s_elevator.getPosition();
    }
}
