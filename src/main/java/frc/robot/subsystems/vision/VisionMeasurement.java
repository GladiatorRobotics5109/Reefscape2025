package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public record VisionMeasurement(String cameraName, Pose2d estimatedPose, double timestamp, Matrix<N3, N1> stdDevs) {
    public static VisionMeasurement[] fromInputs(VisionIOInputs inputs, Matrix<N3, N1> stdDevs) {
        VisionMeasurement[] measurements = new VisionMeasurement[inputs.poses.length];

        for (int i = 0; i < inputs.poses.length; i++) {
            measurements[i] = new VisionMeasurement(
                inputs.cameraName,
                inputs.poses[i].toPose2d(),
                inputs.timestamps[i],
                stdDevs
            );
        }

        return measurements;
    }
}
