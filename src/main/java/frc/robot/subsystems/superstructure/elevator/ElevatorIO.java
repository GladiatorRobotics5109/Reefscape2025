package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double accelerationRadPerSecPerSec = 0.0;

        public double motorTempCelsius = 0.0;
        public double motorAppliedVolts = 0.0;
        public double motorSupplyVoltage = 0.0;
        public double motorStatorCurrentAmps = 0.0;
        public double motorSupplyCurrentAmps = 0.0;
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setPosition(double motorPositionRad) {}

    default void setVoltage(double volts) {}
}
