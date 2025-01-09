package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public record VisionMeasurement(Pose3d estimatedPose, double timestmap, Pose3d[] targetsUsed) {
    public static List<VisionMeasurement> fromInputs(VisionIOInputs inputs) {
        List<VisionMeasurement> measurements = new ArrayList<>(inputs.posees.size());

        for (int i = 0; i < inputs.posees.size(); i++) {
            measurements.add(new VisionMeasurement(inputs.posees.get(i), inputs.timestamps.get(i), inputs.targetsUsed.toArray(new Pose3d[0])));
        }

        return measurements;
    }
}
