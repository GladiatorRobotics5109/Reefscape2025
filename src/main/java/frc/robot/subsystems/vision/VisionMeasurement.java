package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public record VisionMeasurement(String cameraName, Pose2d estimatedPose, double timestamp) {
    public static VisionMeasurement[] fromInputs(VisionIOInputs inputs) {
        VisionMeasurement[] measurements = new VisionMeasurement[inputs.poses.length];

        for (int i = 0; i < inputs.poses.length; i++) {
            measurements[i] = new VisionMeasurement(
                inputs.cameraName,
                inputs.poses[i].toPose2d(),
                inputs.timestamps[i]
            );
        }

        return measurements;
    }
}
