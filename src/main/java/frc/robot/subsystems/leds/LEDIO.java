package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
    @AutoLog
    public static final class LEDIOInputs {
        public double voltage5V = 0.0;
        public double outputCurrentAmps = 0.0;
        public double supplyVoltage = 0.0;
        public double tempCelius = 0.0;
    }
    
    default void updateInputs(LEDIOInputs inputs) {}

    default void setCTREAnimation(Animation anim) {}

    default void setLEDs(Color clr, int start, int count) {}
}
