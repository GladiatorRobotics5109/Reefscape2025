package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Conversions;

public class ElevatorIOSimTalonFX extends ElevatorIOTalonFX {
    private final DCMotorSim m_sim;

    public ElevatorIOSimTalonFX(int motorPort, int followerPort, boolean useFOC) {
        super(motorPort, followerPort, useFOC);

        // TODO: find correct MOI
        m_sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                useFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
                0.002,
                ElevatorConstants.kGearRatio
            ),
            useFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1)
        );
    }

    @Override
    public void updateSim() {
        TalonFXSimState simState = m_motor.getSimState();

        simState.setSupplyVoltage(12);

        m_sim.setInputVoltage(simState.getMotorVoltage());
        m_sim.update(Constants.kLoopPeriodSecs);

        simState.setRawRotorPosition(m_sim.getAngularPositionRotations() * ElevatorConstants.kGearRatio);
        simState.setRotorVelocity(
            Conversions.radiansToRotations(m_sim.getAngularVelocityRadPerSec() * ElevatorConstants.kGearRatio)
        );
    }
}
