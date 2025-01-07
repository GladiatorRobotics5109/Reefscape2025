package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public Pose2d estimatedPose = new Pose2d();
    }

    default void updateInputs(VisionIOInputs inputs) {}
}
