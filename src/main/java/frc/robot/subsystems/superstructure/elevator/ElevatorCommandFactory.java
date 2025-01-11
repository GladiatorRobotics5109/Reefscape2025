package frc.robot.subsystems.superstructure.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.FieldUtil.ReefBranch;

public class ElevatorCommandFactory {
    public static Command toPosition(ElevatorSubsystem elevator, DoubleSupplier positionMeters) {
        return elevator.runOnce(() -> elevator.setPosition(positionMeters.getAsDouble()));
    }

    public static Command toPosition(ElevatorSubsystem elevator, ReefBranch branch) {
        return ElevatorCommandFactory.toPosition(elevator, branch.getBranchPosition()::getY);
    }

    public static Command debugControllerAxis(ElevatorSubsystem elevator, DoubleSupplier axis) {
        return elevator.run(() -> elevator.setVoltage(axis.getAsDouble() * 2));
    }
}
