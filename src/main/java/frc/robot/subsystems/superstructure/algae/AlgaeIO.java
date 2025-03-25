package frc.robot.subsystems.superstructure.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
    @AutoLog
    public static class AlgaeIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;

        public double tempCelsius = 0.0;
        public double appliedVolts = 0.0;
        public double supplyVoltage = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
    }

    default void setVoltage(double volts) {}

    default void setPosition(double positionRad) {}

    default void updateInputs(AlgaeIOInputs inputs) {}
}
