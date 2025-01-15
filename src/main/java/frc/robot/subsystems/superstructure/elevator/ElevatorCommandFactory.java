package frc.robot.subsystems.superstructure.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.FieldConstants.ReefBranch;
import frc.robot.util.FieldConstants.ReefHeight;

public class ElevatorCommandFactory {
    public static Command toPosition(ElevatorSubsystem elevator, DoubleSupplier positionMeters) {
        return elevator.runOnce(() -> elevator.setPosition(positionMeters.getAsDouble()));
    }

    public static Command toPosition(ElevatorSubsystem elevator, ReefHeight height) {
        return ElevatorCommandFactory.toPosition(
            elevator,
            () -> height.getHeight() - ElevatorConstants.kEndEffectorHeightMeters
        );
    }

    public static Command toPosition(ElevatorSubsystem elevator, ReefBranch branch) {
        return ElevatorCommandFactory.toPosition(
            elevator,
            branch.getHeight()
        );
    }

    public static Command debugControllerAxis(ElevatorSubsystem elevator, DoubleSupplier axis) {
        return elevator.run(() -> elevator.setVoltage(axis.getAsDouble() * 2));
    }
}
