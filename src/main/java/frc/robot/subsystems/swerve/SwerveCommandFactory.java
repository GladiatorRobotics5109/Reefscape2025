package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.SwerveDriveConfiguration;
import frc.robot.subsystems.swerve.swervemodule.SwerveModule;
import frc.robot.util.FieldConstants.ReefBranch;

public final class SwerveCommandFactory {
    private SwerveCommandFactory() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static final class SwerveTeleopCommand extends Command {
        private final SwerveSubsystem m_swerve;
        private final SwerveDriveConfiguration m_config;
        private final DoubleSupplier m_x, m_y, m_rot, m_superSpeed;

        private final SlewRateLimiter m_xRateLimit, m_yRateLimit, m_rotRateLimit;

        public SwerveTeleopCommand(
            SwerveSubsystem swerve,
            SwerveDriveConfiguration config,
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier rot,
            DoubleSupplier superSpeed
        ) {
            m_swerve = swerve;
            addRequirements(m_swerve);
            m_config = config;
            m_x = x;
            m_y = y;
            m_rot = rot;
            m_superSpeed = superSpeed;

            m_xRateLimit = new SlewRateLimiter(20);
            m_yRateLimit = new SlewRateLimiter(20);
            m_rotRateLimit = new SlewRateLimiter(10);

            // Configure button bindings
        }

        @Override
        public void initialize() {

        }

        @Override
        public void execute() {
            double driveSpeedMetersPerSec = m_config.evalDriveSpeed(m_superSpeed.getAsDouble()).in(
                Units.MetersPerSecond
            );
            double vx = driveSpeedMetersPerSec
                * m_xRateLimit.calculate(MathUtil.applyDeadband(m_x.getAsDouble(), Constants.kJoystickDeadzone));
            double vy = driveSpeedMetersPerSec
                * m_yRateLimit.calculate(MathUtil.applyDeadband(m_y.getAsDouble(), Constants.kJoystickDeadzone));

            double rotationSpeedRadPerSec = m_config.evalRotationSpeed(m_superSpeed.getAsDouble()).in(
                Units.RadiansPerSecond
            );
            double vrot = rotationSpeedRadPerSec
                * -m_rotRateLimit.calculate(MathUtil.applyDeadband(m_rot.getAsDouble(), Constants.kJoystickDeadzone));

            m_swerve.drive(vx, vy, vrot, m_config.fieldRelative());
            // m_swerve.drive(0, 1.5, 0, m_config.fieldRelative());
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            m_swerve.drive(0, 0, 0, m_config.fieldRelative());
        }
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

    public static Command makeTeleop(
        SwerveSubsystem swerve,
        DoubleSupplier translateX,
        DoubleSupplier translateY,
        DoubleSupplier rot,
        DoubleSupplier superSpeed
    ) {
        return new SwerveTeleopCommand(
            swerve,
            SwerveConstants.kTeleopConfig,
            translateX,
            translateY,
            rot,
            superSpeed
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

    public static Command driveToReefScore(SwerveSubsystem swerve, ReefBranch branch) {
        return Commands.sequence(
            Commands.print("Started!"),
            driveToPoseThenFollowPath(SwerveConstants.kPPPathFindConstraints, branch.getInnerPath()),
            Commands.print("Done!")
        );
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
