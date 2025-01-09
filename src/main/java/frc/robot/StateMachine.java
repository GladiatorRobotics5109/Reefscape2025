package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionMeasurement;
import frc.robot.subsystems.vision.VisionSubsystem;

public class StateMachine {
    private static SwerveSubsystem s_swerve;
    private static VisionSubsystem s_vision;

    public static void init(SwerveSubsystem swerve, VisionSubsystem vision) {
        s_swerve = swerve;
        s_vision = vision;
    }

    public static VisionMeasurement[] getVisionMeasurements() {
        return s_vision.getMeasurements();
    }

    public static Pose2d getPose() {
        return s_swerve.getPose();
    }
}
