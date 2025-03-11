package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public record VisionMeasurement(String cameraName, Pose3d estimatedPose, double timestamp) {
    public static VisionMeasurement[] fromInputs(VisionIOInputs inputs) {
        VisionMeasurement[] measurements = new VisionMeasurement[inputs.poses.length];

        for (int i = 0; i < inputs.poses.length; i++) {
            measurements[i] = new VisionMeasurement(inputs.cameraName, inputs.poses[i], inputs.timestamps[i]);
        }

        return measurements;
    }
}
