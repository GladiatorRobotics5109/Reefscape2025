package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public Pose3d[] poses;
        public double[] timestamps;
        public Pose3d[][] targetsUsed;
    }

    default void updateInputs(VisionIOInputs inputs) {}
}
