package frc.robot.util;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.UtilBase;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.Mode;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public final class Util extends UtilBase {
    public static Alliance getAlliance() { return Util.getAlliance(Constants.kDefaultAlliance); }

    public static boolean isSim() { return Constants.kCurrentMode == Mode.SIM; }

    public static boolean isReal() { return Constants.kCurrentMode == Mode.REAL; }

    public static boolean isReplay() { return Constants.kCurrentMode == Mode.REPLAY; }
}
