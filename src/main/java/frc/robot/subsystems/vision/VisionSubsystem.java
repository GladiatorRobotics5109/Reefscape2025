package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase {
    private VisionIO[] m_ios;
    private VisionIOInputs[] m_inputs;

    private List<VisionMeasurement> m_measurements;

    public VisionSubsystem() {
        m_ios = new VisionIO[0];

        switch (Constants.kCurrentMode) {
            case REAL:
                for (int i = 0; i < VisionConstants.kCameras.length; i++) {
                    m_ios[i] = new VisionIOPhotonVision(VisionConstants.kCameras[i]);
                }

                break;
            default:
                break;
        }

        m_measurements = new ArrayList<>();
    }

    public VisionMeasurement[] getMeasurements() {
        VisionMeasurement[] measurements = m_measurements.toArray(new VisionMeasurement[0]);
        m_measurements.clear();

        return measurements;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < m_ios.length; i++) {
            m_ios[i].updateInputs(m_inputs[i]);

            m_measurements.addAll(VisionMeasurement.fromInputs(m_inputs[i]));
        }

        Logger.recordOutput(
            VisionConstants.kLogPath.concat("measurements"),
            m_measurements.toArray(new VisionMeasurement[0])
        );
    }
}
