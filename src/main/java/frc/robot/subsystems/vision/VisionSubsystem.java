package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class VisionSubsystem extends SubsystemBase {
    private final VisionIO[] m_ios;
    private final VisionIOInputs[] m_inputs;

    private final Queue<VisionMeasurement> m_measurements;

    public VisionSubsystem() {
        m_ios = new VisionIO[VisionConstants.kCameras.length];
        m_inputs = new VisionIOInputs[VisionConstants.kCameras.length];

        switch (Constants.kCurrentMode) {
            // case REAL:
            //     for (int i = 0; i < VisionConstants.kCameras.length; i++) {
            //         m_ios[i] = new VisionIOPhotonVision(VisionConstants.kCameras[i]);
            //         m_inputs[i] = new VisionIOInputs();
            //     }

            //     break;
            default:
                for (int i = 0; i < VisionConstants.kCameras.length; i++) {
                    m_ios[i] = new VisionIO() {};
                    m_inputs[i] = new VisionIOInputs();
                }
                break;
        }

        m_measurements = new LinkedList<>();
    }

    public VisionMeasurement[] getMeasurements() {
        VisionMeasurement[] measurements = new VisionMeasurement[m_measurements.size()];
        m_measurements.removeAll(List.of(measurements));
        
        return measurements;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < m_ios.length; i++) {
            m_ios[i].updateInputs(m_inputs[i]);

            m_measurements.addAll(List.of(VisionMeasurement.fromInputs(m_inputs[i])));
        }

        // TODO: figure out why no log targets here :(
        //        Logger.recordOutput(
        //            VisionConstants.kLogPath.concat("/Measurements"),
        //            m_measurements.toArray(new VisionMeasurement[0])
        //        );
    }
}
