package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIOCANdle implements LEDIO {
    private final CANdle m_candle;

    private Animation m_anim;

    public LEDIOCANdle(int port) {
        m_candle = new CANdle(port);
        m_candle.configLEDType(LEDStripType.GRB);
        m_candle.configVBatOutput(VBatOutputMode.Off);
        m_candle.configV5Enabled(true);
    }

    @Override
    public void updateInputs(LEDIOInputs inputs) {
        inputs.voltage5V = m_candle.get5VRailVoltage();
        inputs.outputCurrentAmps = m_candle.getCurrent();
        inputs.supplyVoltage = m_candle.getBusVoltage();
        inputs.tempCelius = m_candle.getTemperature();
    }

    @Override
    public void setCTREAnimation(Animation anim) {
        m_anim = anim;
        m_candle.animate(m_anim);
    }

    @Override
    public void setLEDs(Color clr, int start, int count) {
        m_candle.setLEDs((int)(255 * clr.red), (int)(clr.green), (int)(clr.blue), 255, start, count);
    }
}
