package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;

        public double tempCelsius = 0.0;
        public double appliedVolts = 0.0;
        public double supplyVoltage = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
    }

    default void updateInputs(ClimbIOInputs inputs) {}

    default void setVoltage(double volts) {}
}
