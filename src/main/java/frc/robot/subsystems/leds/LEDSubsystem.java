package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
    private final LEDIO m_io;
    private final LEDIOInputsAutoLogged m_inputs;

    public LEDSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_io = new LEDIOCANdle(LEDConstants.kCANdlePort);

                break;
            default:
                m_io = new LEDIO() {};

                break;
        }

        m_inputs = new LEDIOInputsAutoLogged();
    }

    public void setCTREAnimation(Animation anim) {
        m_io.setCTREAnimation(anim);
    }

    public void setLEDs(Color clr, int start, int count) {
        m_io.setLEDs(clr, start, count);
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(LEDConstants.kLogPath, m_inputs);
    }
}
