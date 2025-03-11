package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public Pose3d[] poses = new Pose3d[0];
        public double[] timestamps = new double[0];
        public Pose3d[][] targetsUsed = new Pose3d[0][];
        public String cameraName = "";
    }

    default void updateInputs(VisionIOInputs inputs) {}
}
