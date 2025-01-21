package frc.robot.subsystems.superstructure.endeffector;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
    // See LoggedDigitalInput in com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggeddigitalinput.LoggedDigitalInput
    // for box sensors

    public void setVoltage(double leftVolts, double rightVolts) {

    }

    public void setVoltage(double volts) {
        setVoltage(volts, volts);
    }

    public void setScore() {
        setVoltage(EndEffectorConstants.kScoreVoltage);
    }

    public void stop() {
        setVoltage(0);
    }

    public boolean hasCoral() {
        // TODO: implement this
        return false;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            stop();
        }
    }
}
