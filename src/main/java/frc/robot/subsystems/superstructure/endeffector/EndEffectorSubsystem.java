package frc.robot.subsystems.superstructure.endeffector;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
    // See LoggedDigitalInput in com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggeddigitalinput.LoggedDigitalInput
    // for box sensors

    private final SparkMax m_leftMotor = new SparkMax(0, MotorType.kBrushless);
    private final SparkMax m_rightMotor = new SparkMax(0, MotorType.kBrushless);

    public void setPower(double leftVolts, double rightVolts) {

    }

    public void setPower(double volts) {
        setPower(volts, volts);
    }

    //outtake power setter
    public void setScore() {
        setPower(EndEffectorConstants.kScoreVoltage);
    }

    public void stop() {
        setPower(0);
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
