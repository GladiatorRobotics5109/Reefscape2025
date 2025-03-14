package frc.robot.util;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.math.ConversionsBase;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveConstants;

public class Conversions extends ConversionsBase {
    public static double driveWheelMetersToDriveMotorRadians(double m) {
        return ConversionsBase.driveWheelMetersToDriveMotorRadians(
            m,
            SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters,
            SwerveConstants.SwerveModuleConstants.kDriveGearRatio
        );
    }

    public static double driveWheelMetersToWheelRadians(double m) {
        return ConversionsBase.driveWheelMetersToWheelRadians(
            m,
            SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters
        );
    }

    public static double driveWheelRadiansToWheelMeters(double m) {
        return ConversionsBase.driveWheelRadiansToWheelMeters(
            m,
            SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters
        );
    }

    public static double driveMotorRotationsToDriveWheelRadians(double rot) {
        return ConversionsBase.driveMotorRotationsToDriveWheelRadians(
            rot,
            SwerveConstants.SwerveModuleConstants.kDriveGearRatio
        );
    }

    public static double driveMotorRadiansToDriveWheelMeters(double rad) {
        return ConversionsBase.driveMotorRadiansToDriveWheelMeters(
            rad,
            SwerveConstants.SwerveModuleConstants.kWheelRadiusMeters,
            SwerveConstants.SwerveModuleConstants.kDriveGearRatio
        );
    }

    public static Rotation2d driveWheelAngleRotation2dToTurnMotorRotation2d(Rotation2d angle) {
        return ConversionsBase.driveWheelAngleRotation2dToTurnMotorRotation2d(
            angle,
            SwerveConstants.SwerveModuleConstants.kTurnGearRatio
        );
    }

    public static double driveWheelRotationsToDriveMotorRadians(double rot) {
        return ConversionsBase.driveWheelRotationsToDriveMotorRadians(
            rot,
            SwerveConstants.SwerveModuleConstants.kDriveGearRatio
        );
    }

    public static double elevatorMetersToElevatorRotations(double m) {
        return ConversionsBase.radiansToRotations(
            ConversionsBase.metersToRadians(m, ElevatorConstants.kSprocketRadiusMeters, 1.0)
        );
    }

    public static double elevatorMetersToElevatorRadians(double m) {
        return ConversionsBase.metersToRadians(m, ElevatorConstants.kSprocketRadiusMeters, 1.0);
    }

    public static double elevatorRadiansToElevatorMeters(double rad) {
        return ConversionsBase.radiansToMeters(rad, ElevatorConstants.kSprocketRadiusMeters) * 2;
    }

    public static double elevatorRotationsToElevatorMeters(double rot) {
        return elevatorRadiansToElevatorMeters(ConversionsBase.rotationsToRadians(rot));
    }

    public static double elevatorMetersToEndEffectorMeters(double m) {
        return m + EndEffectorConstants.kEndEffectorLengthMeters;
    }

    public static double endEffectorMetersToElevatorMeters(double m) {
        return m - EndEffectorConstants.kEndEffectorLengthMeters;
    }
}
