package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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

    default Matrix<N3, N1> getStdDevs() { return MatBuilder.fill(Nat.N3(), Nat.N1(), 0.0, 0.0, 0.0); }
}
