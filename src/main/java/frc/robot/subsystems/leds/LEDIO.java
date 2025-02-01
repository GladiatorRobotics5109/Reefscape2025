package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.wpilibj.util.Color;

public interface LEDIO {
    //    @AutoLog
    public static final class LEDIOInputs {
        //        public static
    }

    default void setCTREAnimation(Animation anim) {}

    default void setLEDs(Color clr, int start, int count) {}
}
