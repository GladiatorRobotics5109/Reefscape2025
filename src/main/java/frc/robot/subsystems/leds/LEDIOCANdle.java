package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIOCANdle implements LEDIO {
    private final CANdle m_candle;

    private Animation m_anim;

    public LEDIOCANdle(int port) {
        m_candle = new CANdle(port);
    }

    @Override
    public void setCTREAnimation(Animation anim) {
        m_candle.animate(anim);
    }

    @Override
    public void setLEDs(Color clr, int start, int count) {
        m_candle.setLEDs((int)(255 * clr.red), (int)(clr.green), (int)(clr.blue), 255, start, count);
    }
}
