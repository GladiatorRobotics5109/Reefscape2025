package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

public class VisionSubsystem extends SubsystemBase {
    private final VisionIO[] m_ios;
    private final VisionIOInputsAutoLogged[] m_inputs;

    private final Consumer<VisionMeasurement[]> m_addVisionMeasurements;

    public VisionSubsystem(Consumer<VisionMeasurement[]> addVisionMeasurements) {
        m_ios = new VisionIO[VisionConstants.kCameras.length];
        m_inputs = new VisionIOInputsAutoLogged[VisionConstants.kCameras.length];
        m_addVisionMeasurements = addVisionMeasurements;

        switch (Constants.kCurrentMode) {
            //             case REAL:
            //                 for (int i = 0; i < VisionConstants.kCameras.length; i++) {
            //                     m_ios[i] = new VisionIOPhotonVision(VisionConstants.kCameras[i]);
            //                     m_inputs[i] = new VisionIOInputs();
            //                 }
            //
            //                 break;
//            case SIM:
//                for (int i = 0; i < VisionConstants.kCameras.length; i++) {
//                    m_ios[i] = new VisionIOSim();
//                    m_inputs[i] = new VisionIOInputsAutoLogged();
//                }
//
//                break;
            default:
                for (int i = 0; i < VisionConstants.kCameras.length; i++) {
                    m_ios[i] = new VisionIO() {};
                    m_inputs[i] = new VisionIOInputsAutoLogged();
                }

                break;
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < m_ios.length; i++) {
            m_ios[i].updateInputs(m_inputs[i]);
            Logger.processInputs(VisionConstants.kLogPath + "/" + m_inputs[i].cameraName, m_inputs[i]);
            m_addVisionMeasurements.accept(VisionMeasurement.fromInputs(m_inputs[i]));
        }
    }
}
