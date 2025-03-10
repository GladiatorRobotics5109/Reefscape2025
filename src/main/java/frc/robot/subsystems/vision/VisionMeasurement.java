package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public record VisionMeasurement(Pose3d estimatedPose, double timestamp, Pose3d[] targetsUsed) {
    public static VisionMeasurement[] fromInputs(VisionIOInputs inputs) {
        VisionMeasurement[] measurements = new VisionMeasurement[inputs.poses.length];

        for (int i = 0; i < inputs.poses.length; i++) {
            measurements[i] = new VisionMeasurement(inputs.poses[i], inputs.timestamps[i], inputs.targetsUsed[i]);
        }

        return measurements;
    }
}
