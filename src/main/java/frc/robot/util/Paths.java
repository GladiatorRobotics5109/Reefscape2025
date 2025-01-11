package frc.robot.util;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;

public final class Paths {
    public static PathPlannerPath L2E1;

    public static void init() {
        try {
            L2E1 = PathPlannerPath.fromPathFile("InnerL2E1");
        }
        catch (Exception e) {
            DriverStation.reportError("Failed to load path!\n" + e.getMessage(), e.getStackTrace());
        }
    }
}
