package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    public static class VisionIOInputs implements LoggableInputs {
        public List<Pose3d> posees = new ArrayList<>();
        public List<Double> timestamps = new ArrayList<>();
        public Set<Pose3d> targetsUsed = new HashSet<>();

        @Override
        public void toLog(LogTable table) {
            Pose3d[] poses = posees.toArray(new Pose3d[0]);
            table.put("Poses", poses);

            double[] timestampsArray = new double[timestamps.size()];
            for (int i = 0; i < timestampsArray.length; i++) {
                timestampsArray[i] = timestamps.get(i);
            }
            table.put("Timestamps", timestampsArray);

            Pose3d[] targets = targetsUsed.toArray(new Pose3d[0]);
            table.put("TargetsUsed", targets);
        }

        @Override
        public void fromLog(LogTable table) {
            posees = Arrays.asList(table.get("Poses", new Pose3d[0]));

            // Could these be a one-liner??
            double[] timestampsArray = table.get("Timestamps", new double[0]);
            Double[] timestampsDoubleArray = new Double[timestampsArray.length];
            for (int i = 0; i < timestampsDoubleArray.length; i++) {
                timestampsDoubleArray[i] = timestampsArray[i];
            }
            timestamps = Arrays.asList(timestampsDoubleArray);

            Pose3d[] targetsUsedArray = table.get("TargetsUsed", new Pose3d[0]);
            for (int i = 0; i < targetsUsedArray.length; i++) {
                targetsUsed.add(targetsUsedArray[i]);
            }
        }
    }

    default void updateInputs(VisionIOInputs inputs) {}
}
