package frc.robot.subsystems.superstructure.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs {
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;

        public double leftTempCelsius = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftSupplyVoltage = 0.0;
        public double leftStatorCurrentAmps = 0.0;
        public double leftSupplyCurrentAmps = 0.0;

        public double rightPositionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;

        public double rightTempCelsius = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightSupplyVoltage = 0.0;
        public double rightStatorCurrentAmps = 0.0;
        public double rightSupplyCurrentAmps = 0.0;
    }

    default void updateInputs(EndEffectorIOInputs inputs) {}

    default void setVoltage(double leftVolts, double rightVolts) {}
}
