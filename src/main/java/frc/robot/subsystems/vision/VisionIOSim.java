package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants.VisionConstants;

public class VisionIOSim implements VisionIO {
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.poses = new Pose3d[] { new Pose3d(5, 5, 1, Rotation3d.kZero) };
        inputs.timestamps = new double[] { 0.0 };
        inputs.targetsUsed = new Pose3d[][] { { VisionConstants.kAprilTagFieldLayout.getTagPose(5).orElse(
            Pose3d.kZero
        ) } };

        inputs.cameraName = "Sim";
    }
}
