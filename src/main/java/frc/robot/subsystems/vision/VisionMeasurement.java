package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionMeasurement {
    public static record VisionPosition(Pose2d position, String cameraName, double timestamp) {}

    public VisionPosition[] m_positions;

    public VisionMeasurement(VisionPosition[] positions) {
        m_positions = positions;
    }

    public VisionPosition[] getPositions() {
        return m_positions;
    }
}
