package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;
import frc.robot.util.FieldConstants.ReefConstants.ReefHeight;
import org.littletonrobotics.junction.Logger;

public class ElevatorCommandFactory {
    public static Command toElevatorRelativeHeight(ElevatorSubsystem elevator, DoubleSupplier positionMeters) {
        return elevator.runOnce(() -> elevator.setTargetHeightElevatorRelative(positionMeters.getAsDouble()));
    }

    public static Command toReefHeight(ElevatorSubsystem elevator, ReefHeight height) {
        return elevator.runOnce(() -> elevator.setTargetHeightFieldRelative(height));
    }

    public static Command toReefBranch(ElevatorSubsystem elevator, ReefBranch branch) {
        return ElevatorCommandFactory.toReefHeight(elevator, branch.getHeight());
    }

    public static Command toHome(ElevatorSubsystem elevator) {
        return toElevatorRelativeHeight(elevator, () -> 0.0);
    }

    public static Command autoToReefBranch(ElevatorSubsystem elevator, ReefBranch branch) {
        return Commands.waitUntil(elevator::isWithinRadius).withTimeout(2).andThen(toReefBranch(elevator, branch));
    }

    public static Command debugControllerAxis(ElevatorSubsystem elevator, DoubleSupplier axis) {
        return elevator.run(() -> elevator.setVoltage(axis.getAsDouble() * 2));
    }

    public static Command makeSysId(ElevatorSubsystem elevator) {
        var routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())
            ),
            new SysIdRoutine.Mechanism(voltage -> elevator.setVoltage(voltage.in(Units.Volts)), null, elevator)
        );

        return Commands.sequence(
            routine.quasistatic(SysIdRoutine.Direction.kForward),
            routine.quasistatic(SysIdRoutine.Direction.kReverse),
            routine.dynamic(SysIdRoutine.Direction.kForward),
            routine.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.print("Finished!")
        );
    }
}
