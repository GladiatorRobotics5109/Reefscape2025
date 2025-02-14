package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class ControllerRumbleCommand extends Command {
    public static Command linearDecayCommand(
        double startValue,
        RumbleType type,
        double lengthSeconds,
        GenericHID[] targets
    ) {
        return new ControllerRumbleCommand(
            (Double t) -> MathUtil.interpolate(startValue, 0.0, t / lengthSeconds),
            type,
            lengthSeconds,
            targets
        ).withName("ControllerRumbleCommand/LinearDecay");
    }

    private final Function<Double, Double> m_valueSupplier;
    private final RumbleType m_type;
    private final double m_lengthSeconds;
    private final Timer m_timer;

    private final GenericHID[] m_targets;

    public ControllerRumbleCommand(
        Function<Double, Double> valueSupplier,
        RumbleType type,
        double lengthSeconds,
        GenericHID[] targets
    ) {
        setName("ControllerRumbleCommand");

        m_valueSupplier = valueSupplier;
        m_type = type;
        m_lengthSeconds = lengthSeconds;
        m_timer = new Timer();
        m_targets = targets;
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        double t = m_valueSupplier.apply(m_timer.get());

        for (GenericHID target : m_targets) {
            target.setRumble(m_type, t);
        }
    }

    @Override
    public boolean isFinished() { return m_timer.hasElapsed(m_lengthSeconds); }
}
