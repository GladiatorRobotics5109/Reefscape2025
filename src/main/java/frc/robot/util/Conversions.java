package frc.robot.util;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.math.ConversionsBase;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveConstants;

public class Conversions extends ConversionsBase {
    public static final double driveWheelMetersToDriveMotorRadians(double m) {
        return ConversionsBase.driveWheelMetersToDriveMotorRadians(
            m,
            SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters,
            SwerveConstants.SwerveModuleConstants.kDriveGearRatio
        );
    }

    public static final double driveWheelMetersToWheelRadians(double m) {
        return ConversionsBase.driveWheelMetersToWheelRadians(
            m,
            SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters
        );
    }

    public static final double driveWheelRadiansToWheelMeters(double m) {
        return ConversionsBase.driveWheelRadiansToWheelMeters(
            m,
            SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters
        );
    }

    public static final double driveMotorRotationsToDriveWheelRadians(double rot) {
        return ConversionsBase.driveMotorRotationsToDriveWheelRadians(
            rot,
            SwerveConstants.SwerveModuleConstants.kDriveGearRatio
        );
    }

    public static final double driveMotorRadiansToDriveWheelMeters(double rad) {
        return ConversionsBase.driveMotorRadiansToDriveWheelMeters(
            rad,
            SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters,
            SwerveConstants.SwerveModuleConstants.kDriveGearRatio
        );
    }

    public static final Rotation2d driveWheelAngleRotation2dToTurnMotorRotation2d(Rotation2d angle) {
        return ConversionsBase.driveWheelAngleRotation2dToTurnMotorRotation2d(
            angle,
            SwerveConstants.SwerveModuleConstants.kTurnGearRatio
        );
    }

    public static final double driveWheelRotationsToDriveMotorRadians(double rot) {
        return ConversionsBase.driveWheelRotationsToDriveMotorRadians(
            rot,
            SwerveConstants.SwerveModuleConstants.kDriveGearRatio
        );
    }

    // TODO: check this
    public static final double elevatorPositionToRotations(double m) {
        return ConversionsBase.radiansToRotations(
            ConversionsBase.metersToRadians(m, ElevatorConstants.kSproketRadiusMeters, 1.0)
        );
    }

    public static final double elevatorRotationsToElevatorPosition(double rot) {
        return ConversionsBase.radiansToMeters(
            ConversionsBase.rotationsToRadians(rot),
            ElevatorConstants.kSproketRadiusMeters
        );
    }
}
