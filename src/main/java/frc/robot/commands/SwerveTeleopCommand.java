package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.SwerveDriveConfiguration;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public final class SwerveTeleopCommand extends Command {
    private final SwerveSubsystem m_swerve;
    private final SwerveDriveConfiguration m_config;
    private final DoubleSupplier m_x, m_y, m_rot, m_speedModifier;

    private final SlewRateLimiter m_xRateLimit, m_yRateLimit, m_rotRateLimit;

    public SwerveTeleopCommand(
        SwerveSubsystem swerve,
        SwerveDriveConfiguration config,
        DoubleSupplier x,
        DoubleSupplier y,
        DoubleSupplier rot,
        DoubleSupplier speedModifier
    ) {
        m_swerve = swerve;
        addRequirements(m_swerve);
        m_config = config;
        m_x = x;
        m_y = y;
        m_rot = rot;
        m_speedModifier = speedModifier;

        m_xRateLimit = new SlewRateLimiter(20);
        m_yRateLimit = new SlewRateLimiter(20);
        m_rotRateLimit = new SlewRateLimiter(10);

        setName("SwerveTeleopCommand");
    }

    @Override
    public void execute() {
        boolean fieldRelative = m_config.fieldRelative();
        double driveSpeedMetersPerSec = m_config.evalDriveSpeed(m_speedModifier.getAsDouble());
        double vx = driveSpeedMetersPerSec
            * m_xRateLimit.calculate(
                MathUtil.applyDeadband(
                    fieldRelative ? m_y.getAsDouble() : m_x.getAsDouble(),
                    Constants.kJoystickDeadzone
                )
            );
        double vy = driveSpeedMetersPerSec
            * m_yRateLimit.calculate(
                MathUtil.applyDeadband(
                    fieldRelative ? m_x.getAsDouble() : m_y.getAsDouble(),
                    Constants.kJoystickDeadzone
                )
            );

        double rotationSpeedRadPerSec = m_config.evalRotationSpeed(m_speedModifier.getAsDouble());
        double vrot = rotationSpeedRadPerSec
            * -m_rotRateLimit.calculate(
                MathUtil.applyDeadband(m_rot.getAsDouble(), Constants.kJoystickDeadzone)
            );

        m_swerve.drive(vx, vy, vrot, fieldRelative);
        // m_swerve.drive(0.0, 1.0, 0.0, false);
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(0, 0, 0, m_config.fieldRelative());
    }
}
