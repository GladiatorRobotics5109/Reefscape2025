package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.swervemodule.SwerveModule;
import frc.robot.util.Conversions;
import frc.robot.util.FieldConstants.ReefConstants.ReefBranch;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class SwerveCommandFactory {
    private SwerveCommandFactory() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static Command makeTeleop(SwerveSubsystem swerve, CommandXboxController controller) {
        return new SwerveTeleopCommand(
            swerve,
            SwerveConstants.kTeleopConfig,
            () -> -controller.getLeftX(),
            () -> -controller.getLeftY(),
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
            () -> -controller.getLeftX(),
            () -> -controller.getLeftY(),
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

    public static Command drive(SwerveSubsystem swerve, double vx, double vy, double vrot, boolean fieldRelative) {
        return swerve.runOnce(() -> swerve.drive(vx, vy, vrot, fieldRelative));
    }

    public static Command stopAndX(SwerveSubsystem swerve) {
        return swerve.runOnce(swerve::stopAndX);
    }

    public static Command followPath(SwerveSubsystem swerve, PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    public static Command followPathControllerInfluence(
        SwerveSubsystem swerve,
        PathPlannerPath path,
        DoubleSupplier x,
        DoubleSupplier y,
        DoubleSupplier rot
    ) {
        double driveSpeed = SwerveConstants.kTeleopConfig.defaultDriveSpeed().in(Units.MetersPerSecond);
        double rotSpeed = SwerveConstants.kTeleopConfig.defaultRotationSpeed().in(Units.RadiansPerSecond);

        return new FollowPathCommand(
            path,
            swerve::getPose,
            swerve::getCurrentChassisSpeeds,
            (speeds, feedForward) -> swerve.drive(
                speeds.plus(
                    new ChassisSpeeds(
                        x.getAsDouble() * driveSpeed,
                        y.getAsDouble() * driveSpeed,
                        rot.getAsDouble() * rotSpeed
                    )
                ),
                SwerveConstants.kTeleopFieldRelative
            ),
            new PPHolonomicDriveController(SwerveConstants.kPPTranslationPID, SwerveConstants.kPPRotaitonPID),
            SwerveConstants.kPPConfig,
            () -> Util.getAlliance() == Alliance.Red,
            swerve
        );
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

    public static Command makeSysIdTurn(SwerveSubsystem swerve) {
        SwerveModule[] modules = swerve.getSwerveModules();

        Timer timer = new Timer();
        final double kRampRateVoltsPerSec = 0.1;
        final String kLogPath = SwerveConstants.kLogPath + "/SysIdTurn";
        final double kSpeedThresholdRadPerSec = Conversions.degreesToRadians(0.1);

        return Commands.sequence(
            swerve.runOnce(() -> {
                for (var module : modules) {
                    module.setDesiredState(new SwerveModuleState(), false);
                }
            }),
            Commands.waitSeconds(1.5),
            new FunctionalCommand(
                timer::restart, // OnInit
                () -> { // Execute
                    double volts = kRampRateVoltsPerSec * timer.get();
                    for (var module : modules) {
                        module.setTurnVoltage(volts);
                    }

                    Logger.recordOutput(kLogPath + "/AppliedVolts", volts);
                    Logger.recordOutput(kLogPath + "/CurrentTurnVelocities", swerve.getModuleTurnVelocities());
                },
                (Boolean interrupted) -> { // End
                    for (var module : modules) {
                        module.setDesiredState(new SwerveModuleState(), false);
                    }

                    timer.stop();
                },
                () -> { // IsFinished
                    double[] velocities = swerve.getModuleTurnVelocities();
                    double avgSpeed = 0.0;
                    for (double vel : velocities) {
                        avgSpeed += vel;
                    }
                    avgSpeed /= 4;

                    return avgSpeed >= kSpeedThresholdRadPerSec;
                },
                swerve
            )
        );
    }

    public static Command makeSysIdDrive(SwerveSubsystem swerve) {
        SwerveModule[] modules = swerve.getSwerveModules();

        Timer timer = new Timer();
        final double kRampRateVoltsPerSec = 0.1;
        final String kLogPath = SwerveConstants.kLogPath + "/SysIdDrive";
        final double kSpeedThresholdMetersPerSec = Conversions.inchesToMeters(0.1);

        return Commands.sequence(
            swerve.runOnce(() -> {
                for (var module : modules) {
                    module.setDesiredState(new SwerveModuleState(), false);
                }
            }),
            Commands.waitSeconds(1.5),
            new FunctionalCommand(
                timer::restart, // OnInit
                () -> { // Execute
                    double volts = kRampRateVoltsPerSec * timer.get();
                    for (var module : modules) {
                        module.setDriveVoltage(volts);
                    }

                    Logger.recordOutput(kLogPath + "/AppliedVolts", volts);
                    Logger.recordOutput(kLogPath + "/CurrentStates", swerve.getModuleStates());
                },
                (Boolean interrupted) -> { // End
                    for (var module : modules) {
                        module.setDesiredState(new SwerveModuleState(), false);
                    }

                    timer.stop();
                },
                () -> { // IsFinished
                    SwerveModuleState[] states = swerve.getModuleStates();
                    double avgSpeed = 0.0;
                    for (SwerveModuleState state : states) {
                        avgSpeed += state.speedMetersPerSecond;
                    }
                    avgSpeed /= 4;

                    return avgSpeed >= kSpeedThresholdMetersPerSec;
                },
                swerve
            )
        );
    }
}
