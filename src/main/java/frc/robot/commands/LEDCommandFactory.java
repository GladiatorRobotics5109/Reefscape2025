package frc.robot.commands;

import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.leds.LEDSubsystem;

public class LEDCommandFactory {
    public static class LEDRGBCommand extends Command {
        private final LEDSubsystem m_leds;

        private final double m_brightness;
        private final double m_speed;
        private final boolean m_reverse;

        public LEDRGBCommand(LEDSubsystem leds, double brightness, double speed, boolean reverse) {
            m_leds = leds;
            addRequirements(m_leds);

            m_brightness = brightness;
            m_speed = speed;
            m_reverse = reverse;
        }

        @Override
        public void initialize() {
            m_leds.setCTREAnimation(
                new RainbowAnimation(m_brightness, m_speed, Constants.LEDConstants.kLEDCount, m_reverse, 0)
            );
        }
    }

    public static class LEDFieldRelativeRGBCommand extends Command {
        private final LEDSubsystem m_leds;

        private final Rotation2d m_headingOffset;

        public LEDFieldRelativeRGBCommand(LEDSubsystem leds, Rotation2d headingOffset) {
            m_leds = leds;
            addRequirements(m_leds);

            m_headingOffset = headingOffset;
        }

        @Override
        public void execute() {
            for (int i = 0; i < Constants.LEDConstants.kLEDCount; i++) {
                Color clr = Color.fromHSV(
                    (int)(RobotState.getSwervePose().getRotation().plus(m_headingOffset).getDegrees() % 360) / 2,
                    255,
                    255
                );

                m_leds.setLEDs(clr, i, 1);
            }
        }
    }

    public static class LEDStrobeCommand extends Command {
        private final LEDSubsystem m_leds;

        private final Color m_clr;
        private final double m_speed;

        public LEDStrobeCommand(LEDSubsystem leds, Color clr, double speed) {
            m_leds = leds;

            m_clr = clr;
            m_speed = speed;
        }

        @Override
        public void initialize() {
            m_leds.setCTREAnimation(
                new StrobeAnimation(
                    getColorR(m_clr),
                    getColorG(m_clr),
                    getColorB(m_clr),
                    255,
                    m_speed,
                    Constants.LEDConstants.kLEDCount
                )
            );
        }
    }

    public static Command goodThingHappenedCommand(LEDSubsystem leds) {
        final double kStrobeLength = 0.1;
        final int kNumStrobes = 3;

        return new LEDStrobeCommand(leds, Color.kGreen, kStrobeLength).withTimeout(kNumStrobes * kStrobeLength);
    }

    private static int getColorR(Color clr) { return (int)(255 * clr.red); }

    private static int getColorG(Color clr) { return (int)(255 * clr.green); }

    private static int getColorB(Color clr) { return (int)(255 * clr.blue); }
}
