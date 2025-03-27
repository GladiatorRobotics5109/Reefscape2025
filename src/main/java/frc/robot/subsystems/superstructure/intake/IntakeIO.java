package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;

        public double tempCelsius = 0.0;
        public double appliedVolts = 0.0;
        public double supplyVoltage = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setVoltage(double volts) {}
}
