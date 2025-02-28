package frc.robot.subsystems.superstructure.ballsIntake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallsIntakeConstants;

public class ballsIntake extends SubsystemBase {

    private final SparkMax m_pivotMotor = new SparkMax(0, MotorType.kBrushless);
    private final SparkMax m_intakeMotor = new SparkMax(0, MotorType.kBrushless);

    public void setPower(double leftVolts, double rightVolts) {

    }

    public void setPower(double volts) {
        setPower(volts, volts);
    }

    //outtake power setter
    public void setScore() {
        setPower(BallsIntakeConstants.kScorePower);
    }

    public void stop() {
        setPower(0);
    }

    public boolean hasAlgae() {
        // TODO: implement this
        return false;
    }

    //if robot E-Stops or disables, we want to set the desired voltage to zero
    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            stop();
        }
    }

}
