package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    private VisionIO m_io;

    public VisionSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new VisionIOPhotonVision();

                break;
            default:
                m_io = new VisionIO() {
                    
                };

                break;
        }
    }
}
