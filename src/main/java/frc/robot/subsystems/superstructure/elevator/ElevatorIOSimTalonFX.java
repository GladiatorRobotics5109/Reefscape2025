package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Conversions;

public class ElevatorIOSimTalonFX extends ElevatorIOTalonFX {
    private final ElevatorSim m_sim;

    public ElevatorIOSimTalonFX(int motorPort, int followerPort, boolean useFOC) {
        super(motorPort, followerPort, useFOC);

        m_sim = new ElevatorSim(
            DCMotor.getKrakenX60Foc(2),
            ElevatorConstants.kGearRatio,
            Conversions.poundsToKilograms(20),
            ElevatorConstants.kSprocketRadiusMeters,
            0.0,
            ElevatorConstants.kElevatorMaxPositionMeters,
            true,
            0.0,
            0.0,
            0.0
        );
    }

    @Override
    public void updateSim() {
        TalonFXSimState simState = m_motor.getSimState();
        TalonFXSimState followerSimState = m_followerMotor.getSimState();

        simState.setSupplyVoltage(12);
        followerSimState.setSupplyVoltage(12);

        m_sim.setInputVoltage(simState.getMotorVoltage());
        m_sim.update(Constants.kLoopPeriodSecs);

        double rotorPositionRot = Conversions.elevatorMetersToElevatorRotations(m_sim.getPositionMeters())
            * ElevatorConstants.kGearRatio;
        simState.setRawRotorPosition(rotorPositionRot);
        followerSimState.setRawRotorPosition(rotorPositionRot);

        double rotorVelRotPerSec = Conversions.elevatorMetersToElevatorRotations(m_sim.getVelocityMetersPerSecond())
            * ElevatorConstants.kGearRatio;
        simState.setRotorVelocity(rotorVelRotPerSec);
        followerSimState.setRotorVelocity(rotorVelRotPerSec);
    }
}
