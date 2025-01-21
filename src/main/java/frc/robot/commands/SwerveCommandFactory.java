package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.swervemodule.SwerveModule;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;

public final class SwerveCommandFactory {
    private SwerveCommandFactory() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static Command makeTeleop(SwerveSubsystem swerve, CommandXboxController controller) {
        return new SwerveTeleopCommand(
            swerve,
            SwerveConstants.kTeleopConfig,
            controller::getLeftX,
            controller::getLeftY,
            controller::getRightX,
            controller::getRightTriggerAxis
        );
    }

    public static Command makeTeleop(SwerveSubsystem swerve, CommandPS5Controller controller) {
        return new SwerveTeleopCommand(
            swerve,
            SwerveConstants.kTeleopConfig,
            controller::getLeftX,
            controller::getLeftY,
            controller::getRightX,
            controller::getR2Axis
        );
    }

    public static Command makeTeleop(SwerveSubsystem swerve, CommandPS4Controller controller) {
        return new SwerveTeleopCommand(
            swerve,
            SwerveConstants.kTeleopConfig,
            controller::getLeftX,
            controller::getLeftY,
            controller::getRightX,
            controller::getR2Axis
        );
    }

    public static Command makeTeleop(
        SwerveSubsystem swerve,
        DoubleSupplier translateX,
        DoubleSupplier translateY,
        DoubleSupplier rot,
        DoubleSupplier speedModifier
    ) {
        return new SwerveTeleopCommand(
            swerve,
            SwerveConstants.kTeleopConfig,
            translateX,
            translateY,
            rot,
            speedModifier
        );
    }

    public static Command followPath(SwerveSubsystem sewrve, PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    public static Command driveToPose(Pose2d pose, PathConstraints constraints, LinearVelocity endVelocity) {
        return AutoBuilder.pathfindToPoseFlipped(pose, constraints, endVelocity);
    }

    public static Command driveToPoseThenFollowPath(PathConstraints constraints, PathPlannerPath path) {
        return AutoBuilder.pathfindThenFollowPath(path, constraints);
    }

    public static Command setPosition(SwerveSubsystem swerve, Supplier<Pose2d> pose) {
        return swerve.runOnce(() -> swerve.setPosition(pose.get()));
    }

    public static Command driveToReefScore(SwerveSubsystem swerve, ReefBranch branch) {
        return driveToPoseThenFollowPath(SwerveConstants.kPPPathFindConstraints, branch.getInnerPath());
    }

    public static SysIdRoutine makeSysIdTurn(SwerveSubsystem swerve, int modNum) {
        SwerveModule module = swerve.getSwerveModules()[modNum];

        var routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState Mod" + modNum, state.toString())
            ),
            new SysIdRoutine.Mechanism((volts) -> module.setTurnVoltage(volts.in(Units.Volts)), null, swerve)
        );

        return routine;
    }

    public static SysIdRoutine makeSysIdDrive(SwerveSubsystem swerve, int modNum) {
        SwerveModule module = swerve.getSwerveModules()[modNum];

        var routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState Mod" + modNum, state.toString())
            ),
            new SysIdRoutine.Mechanism((volts) -> module.setDriveVoltage(volts.in(Units.Volts)), null, swerve)
        );

        return routine;
    }
}
