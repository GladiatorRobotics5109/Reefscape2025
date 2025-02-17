package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;
import frc.robot.util.FieldConstants.ReefConstants.ReefHeight;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class ElevatorCommandFactory {
    public static Command toElevatorRelativeHeight(ElevatorSubsystem elevator, DoubleSupplier positionMeters) {
        return elevator.runOnce(() -> elevator.setDesiredPositionElevator(positionMeters.getAsDouble()));
    }

    public static Command toReefHeight(ElevatorSubsystem elevator, ReefHeight height) {
        return elevator.runOnce(() -> elevator.setDesiredPositionEndEffector(height));
    }

    public static Command toReefBranch(ElevatorSubsystem elevator, ReefBranch branch) {
        return ElevatorCommandFactory.toReefHeight(elevator, branch.getHeight());
    }

    public static Command toHome(ElevatorSubsystem elevator) {
        return toElevatorRelativeHeight(elevator, () -> 0.0);
    }

    public static Command autoToReefBranch(ElevatorSubsystem elevator, ReefBranch branch) {
        return Commands.waitUntil(elevator::canAutoExtend).withTimeout(2).andThen(toReefBranch(elevator, branch));
    }

    public static Command debugControllerAxis(
        ElevatorSubsystem elevator,
        DoubleSupplier upAxis,
        DoubleSupplier downAxis
    ) {
        return elevator.run(() -> elevator.setVoltage((upAxis.getAsDouble() * 1) - (downAxis.getAsDouble() * 1)));
    }

    public static Command makeSysId(ElevatorSubsystem elevator) {
        var routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Units.Volts.of(3),
                Units.Second.of(1),
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())
            ),
            new SysIdRoutine.Mechanism(voltage -> elevator.setVoltage(voltage.in(Units.Volts)), null, elevator)
        );

        return Commands.sequence(
            routine.quasistatic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(1),
            routine.quasistatic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(1),
            routine.dynamic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(1),
            routine.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.print("Finished!")
        );

        // final double kVoltageRampRate = 0.005;
        // final String kLogPath = ElevatorConstants.kLogPath + "/Sysid";

        // Timer timer = new Timer();

        // return new FunctionalCommand(() -> {
        //     elevator.setVoltage(0.0);
        //     timer.restart();
        // }, () -> {
        //     double appliedVolts = timer.get() * kVoltageRampRate;
        //     Logger.recordOutput(kLogPath + "/AppliedVolts", appliedVolts);
        //     Logger.recordOutput(kLogPath + "/CurrentVelocity", elevator.getCurrentVelocity());
        //     elevator.setVoltage(appliedVolts);
        // }, (interrupted) -> {
        //     elevator.setVoltage(0.0);
        // }, () -> elevator.getCurrentVelocity() > Conversions.elevatorRadiansToElevatorMeters(1.0), elevator);
    }
}
