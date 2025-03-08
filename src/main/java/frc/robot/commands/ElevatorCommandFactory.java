package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.util.Conversions;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;
import frc.robot.util.FieldConstants.ReefConstants.ReefHeight;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class ElevatorCommandFactory {
    public static Command setVoltage(ElevatorSubsystem elevator, double volts) {
        return elevator.runOnce(() -> elevator.setVoltage(volts));
    }

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
        return Commands.sequence(
            elevator.runOnce(elevator::toHome),
            Commands.waitUntil(elevator::atDesiredPosition),
            setVoltage(elevator, -0.2),
            Commands.waitUntil(() -> elevator.getDesiredPositionElevatorRad() <= 0.05),
            elevator.runOnce(elevator::stop)
        );
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
        //        var routine = new SysIdRoutine(
        //            new SysIdRoutine.Config(
        //                null,
        //                Units.Volts.of(3),
        //                Units.Second.of(1),
        //                (state) -> Logger.recordOutput("SysIdTestState", state.toString())
        //            ),
        //            new SysIdRoutine.Mechanism(voltage -> elevator.setVoltage(voltage.in(Units.Volts)), null, elevator)
        //        );
        //
        //        return Commands.sequence(
        //            routine.quasistatic(SysIdRoutine.Direction.kForward),
        //            Commands.waitSeconds(1),
        //            routine.quasistatic(SysIdRoutine.Direction.kReverse),
        //            Commands.waitSeconds(1),
        //            routine.dynamic(SysIdRoutine.Direction.kForward),
        //            Commands.waitSeconds(1),
        //            routine.dynamic(SysIdRoutine.Direction.kReverse),
        //            Commands.print("Finished!")
        //        );

        final double kVoltageRampRate = 0.005;
        final String kLogPath = ElevatorConstants.kLogPath + "/SysId";

        final double[] appliedVolts = { 0.0 };

        Timer timer = new Timer();

        Command up = new FunctionalCommand(() -> {
            elevator.setVoltage(0.0);
            timer.restart();
        }, () -> {
            appliedVolts[0] = timer.get() * kVoltageRampRate;
            Logger.recordOutput(kLogPath + "/AppliedVolts", appliedVolts[0]);
            Logger.recordOutput(kLogPath + "/CurrentVelocity", elevator.getCurrentVelocity());
            elevator.setVoltage(appliedVolts[0]);
        }, (interrupted) -> {
            elevator.setVoltage(0.0);
            timer.stop();
            Logger.recordOutput(kLogPath + "/K_s+K_g", appliedVolts[0]);
            System.out.println("K_s+K_g: " + appliedVolts[0]);
        }, () -> elevator.getCurrentVelocity() > Conversions.elevatorRadiansToElevatorMeters(0.5), elevator);

        Command down = new FunctionalCommand(() -> {
            elevator.setVoltage(0.0);
            appliedVolts[0] = 0.0;
            timer.restart();
        }, () -> {
            appliedVolts[0] = timer.get() * kVoltageRampRate * -1;
            Logger.recordOutput(kLogPath + "/AppliedVolts", appliedVolts);
            Logger.recordOutput(kLogPath + "/CurrentVelocity", elevator.getCurrentVelocity());
            elevator.setVoltage(appliedVolts[0]);
        }, (interrupted) -> {
            elevator.setVoltage(0.0);
            timer.stop();
            Logger.recordOutput(kLogPath + "/K_g-K_s", appliedVolts[0]);
            System.out.println("K_g-K_s: " + appliedVolts[0]);
        }, () -> elevator.getCurrentVelocity() < Conversions.elevatorRadiansToElevatorMeters(-0.5), elevator);

        return Commands.sequence(
            up,
            Commands.runOnce(() -> elevator.setVoltage(1)),
            Commands.waitSeconds(2.5),
            Commands.runOnce(() -> elevator.setVoltage(0.0)),
            down
        );
    }
}
