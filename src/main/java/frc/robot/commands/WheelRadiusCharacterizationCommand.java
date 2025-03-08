package frc.robot.commands;

// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// Modified by 5109

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.function.DoubleSupplier;

public class WheelRadiusCharacterizationCommand extends Command {
    private static final LoggedNetworkNumber characterizationSpeed = new LoggedNetworkNumber(
        "Subsystems/Swerve/WheelRadiusCharacterization/SpeedRadsPerSec",
        0.2
    );
    private static final double driveRadius = SwerveConstants.kDriveBaseRadiusMeters;
    private static final DoubleSupplier gyroYawRadsSupplier = () -> RobotState.getSwervePose().getRotation()
        .getRadians();

    public enum Direction {
        CLOCKWISE(-1),
        COUNTER_CLOCKWISE(1);

        private final int value;

        private Direction(int value) {
            this.value = value;
        }
    }

    private final SwerveSubsystem drive;
    private final Direction omegaDirection;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;

    private double[] startWheelPositions;

    private double currentEffectiveWheelRadius = 0.0;

    public WheelRadiusCharacterizationCommand(SwerveSubsystem drive, Direction omegaDirection) {
        this.drive = drive;
        this.omegaDirection = omegaDirection;
        addRequirements(drive);

        setName("WheelRadiusCharacterization");
    }

    @Override
    public void initialize() {
        // Reset
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        accumGyroYawRads = 0.0;

        startWheelPositions = drive.getWheelRadiusCharacterizationPosition();

        omegaLimiter.reset(0);
    }

    @Override
    public void execute() {
        // Run drive at velocity
        drive.drive(0.0, 0.0, omegaLimiter.calculate(omegaDirection.value * characterizationSpeed.get()), false);

        // Get yaw and wheel positions
        accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        double averageWheelPosition = 0.0;
        double[] wheelPositiions = drive.getWheelRadiusCharacterizationPosition();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
        }
        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
        Logger.recordOutput("Subsystems/Swerve/RadiusCharacterization/DrivePosition", averageWheelPosition);
        Logger.recordOutput("Subsystems/Swerve/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
        Logger.recordOutput(
            "Subsystems/Swerve/RadiusCharacterization/CurrentWheelRadiusInches",
            Units.metersToInches(currentEffectiveWheelRadius)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0.0, 0.0, 0.0, false);
        if (accumGyroYawRads <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
        }
        else {
            System.out.println("Effective Wheel Radius: " + currentEffectiveWheelRadius + " m");
        }
    }

    @Override
    public boolean isFinished() { return accumGyroYawRads >= (Math.PI * 2.0) + (Math.PI / 4); }
}
