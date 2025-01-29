package frc.robot;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.Mode;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.constants.swerveModuleConstants.SwerveDriveSpecialtiesConstants.MK4Constants;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.constants.swerveModuleConstants.SwerveDriveSpecialtiesConstants.MK4Constants.MK4GearRatio;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.math.controller.FeedforwardConstants;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.math.controller.PIDConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.SwerveConstants.SwerveModuleConstants;
import frc.robot.util.Conversions;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.FieldConstants;

public final class Constants {
    public static final Mode kCurrentMode = Mode.SIM;

    public static final Alliance kDefaultAlliance = Alliance.Blue;

    public static final double kLoopPeriodSecs = Robot.defaultPeriodSecs;

    public static final int kPDPPort = 0;

    public static final double kJoystickDeadzone = 0.15;

    public static final double kBumperWidthMeters = Conversions.inchesToMeters(3.25);

    public static final class DriveTeamConstants {
        public static final int kDriveControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class SwerveConstants {
        public static final class SwerveModuleConstants {
            /* Use PID controller on motor controllers */
            public static final boolean kUseMotorPID = true;
            /* Use FOC on TalonFX */
            public static final boolean kUseFOC = false;

            public static final MK4GearRatio kDriveGearRatio = MK4GearRatio.L1;
            public static final double kTurnGearRatio = MK4Constants.kTurnGearRatio;

            public static final double kWheelRadiusMeters = 0.0472659347214289;

            // TODO: get correct ports
            public static final int kFrontLeftDrivePort = 2;
            public static final int kFrontLeftTurnPort = 5;
            public static final int kFrontRightDrivePort = 4;
            public static final int kFrontRightTurnPort = 40;
            public static final int kBackLeftDrivePort = 1;
            public static final int kBackLeftTurnPort = 22;
            public static final int kBackRightDrivePort = 3;
            public static final int kBackRightTurnPort = 30;

            public static final Translation2d kModulePosFL = new Translation2d(0.290449, 0.290449);
            public static final Translation2d kModulePosFR = new Translation2d(0.290449, -0.290449);;
            public static final Translation2d kModulePosBL = new Translation2d(-0.290449, 0.290449);
            public static final Translation2d kModulePosBR = new Translation2d(-0.290449, -0.290449);

            // Units in module space not motor space
            public static final PIDConstants kDrivePID = new PIDConstants(
                0.5, // Volts per m/s error
                0,
                0
            );
            // public static final PIDConstants kDrivePID = new PIDConstants(
            // 0.026251 * kDriveGearRatio.asDouble(), // Volts per m/s error
            // 0,
            // 0
            // );

            // public static final FeedforwardConstants kDriveFeedforward = new FeedforwardConstants(0.15, 9.5 / 3);
            public static final FeedforwardConstants kDriveFeedforward = new FeedforwardConstants(0.14, 6);
            // public static final FeedforwardConstants kDriveFeedForward = new FeedforwardConstants(
            // 0.0091035 * kDriveGearRatio.asDouble(),
            // 0.1546 * kDriveGearRatio.asDouble(),
            // 0.0055267 * kDriveGearRatio.asDouble()
            // );

            public static final PIDConstants kTurnPID = new PIDConstants(
                12 / Conversions.rotationsToRadians(0.01), // Volts per radian error
                0,
                0,
                PIDConstants.kIZone,
                true,
                -Math.PI,
                Math.PI,
                PIDConstants.kPositionTolerance,
                PIDConstants.kVelocityTolerance
            );

            // public static final PIDConstants kTurnPID = new PIDConstants(
            // 0.096535 * kTurnGearRatio,
            // 0,
            // 0,
            // PIDConstants.kIZone,
            // true,
            // -Math.PI,
            // Math.PI,
            // PIDConstants.kPositionTolerance,
            // PIDConstants.kVelocityTolerance
            // );

            public static final FeedforwardConstants kTurnFeedforward = new FeedforwardConstants(0, 0);

            // public static final FeedforwardConstants kTurnFeedforward = new FeedforwardConstants(
            // 0,
            // 0.24103 * kTurnGearRatio,
            // 0.013805 * kTurnGearRatio
            // );

            public static final int kDriveStatorCurrentLimit = 92;
            public static final int kDriveSupplyCurrentLimit = 40;

            public static final int kTurnSupplyCurrentLimit = 30;

            public static final LinearVelocity kDriveMaxFreeSpeed = Units.FeetPerSecond.of(12.9);
            // TODO: verify this
            public static final AngularVelocity kTurnMaxRotationSpeed = Units.RotationsPerSecond.of(2);
        }

        // TODO: change this
        public static final int kPigeonPort = 10;

        public static final boolean kTeleopFieldRelative = true;

        public static final double kOdometryFrequencyHz = 50;
        public static final Pose2d kStartingPose = new Pose2d();
        public static final String kLogPath = "Subsystems/Swerve";
        public static final double kFrameWidth = Conversions.inchesToMeters(29);
        public static final double kFrameHeight = Conversions.inchesToMeters(29);
        public static final double kDriveBaseRadiusMeters = Math.hypot(kFrameWidth, kFrameHeight);

        public static final com.pathplanner.lib.config.PIDConstants kPPTranslationPID = new com.pathplanner.lib.config.PIDConstants(
            5,
            0,
            0
        );
        public static final com.pathplanner.lib.config.PIDConstants kPPRotaitonPID = new com.pathplanner.lib.config.PIDConstants(
            5,
            0,
            0
        );

        // TODO: replace with correct values
        public static final RobotConfig kPPConfig = new RobotConfig(
            52.0,
            5.0,
            // Conversions.poundsToKilograms(112),
            // // Fix rough estimate
            // (1 / 12) * Conversions.poundsToKilograms(112)
            // * (Math.pow(SwerveModuleConstants.kModulePosFL.getX(), 2)
            // + Math.pow(SwerveModuleConstants.kModulePosFL.getY(), 2)),
            new ModuleConfig(
                SwerveModuleConstants.kWheelRadiusMeters,
                SwerveModuleConstants.kDriveMaxFreeSpeed.in(Units.MetersPerSecond),
                1.0,
                SwerveModuleConstants.kUseFOC
                    ? DCMotor.getKrakenX60Foc(1).withReduction(SwerveModuleConstants.kDriveGearRatio.asDouble())
                    : DCMotor.getKrakenX60(1).withReduction(SwerveModuleConstants.kDriveGearRatio.asDouble()),
                SwerveModuleConstants.kDriveSupplyCurrentLimit,
                1
            ),
            SwerveModuleConstants.kModulePosFL,
            SwerveModuleConstants.kModulePosFR,
            SwerveModuleConstants.kModulePosBL,
            SwerveModuleConstants.kModulePosBR
        );

        public static final PathConstraints kPPPathFindConstraints = new PathConstraints(
            Units.MetersPerSecond.of(2),
            Units.MetersPerSecondPerSecond.of(2),
            Units.RotationsPerSecond.of(1.5),
            Units.RotationsPerSecondPerSecond.of(2)
            // Units.MetersPerSecond.of(0.5),
            // Units.MetersPerSecondPerSecond.of(2),
            // Units.RotationsPerSecond.of(0.5),
            // Units.RotationsPerSecondPerSecond.of(2)
        );

        public static final PathConstraints kPPReefInnerPathConstraints = new PathConstraints(
            Units.MetersPerSecond.of(0.25),
            Units.MetersPerSecondPerSecond.of(1),
            Units.RotationsPerSecond.of(1.5),
            Units.RotationsPerSecondPerSecond.of(2)
        );

        public static final PathConstraints kPPCoralStationInnerPathConstraints = kPPReefInnerPathConstraints;

        public static final SwerveDriveConfiguration kTeleopConfig = new SwerveDriveConfiguration(
            SwerveModuleConstants.kDriveMaxFreeSpeed,
            Units.MetersPerSecond.of(0.5),
            // Units.RotationsPerSecond.of(0.2),
            Units.RotationsPerSecond.of(1.25),
            Units.RotationsPerSecond.of(0.5),
            kTeleopFieldRelative
        );

        // Provides a way to describe the configuration of the swerve subsystem (like drive speeds) with values that may
        // change throughout a match (like auto -> teleop)
        public static final record SwerveDriveConfiguration(
            LinearVelocity defaultDriveSpeed,
            LinearVelocity modifiedDriveSpeed,
            AngularVelocity defaultRotationSpeed,
            AngularVelocity modifiedRotationSpeed,
            boolean fieldRelative
        ) {
            public double evalDriveSpeed(double t) {
                return MathUtil.interpolate(
                    defaultDriveSpeed.in(Units.MetersPerSecond),
                    modifiedDriveSpeed.in(Units.MetersPerSecond),
                    t
                );
            }

            public double evalRotationSpeed(double t) {
                return MathUtil.interpolate(
                    defaultRotationSpeed.in(Units.RadiansPerSecond),
                    modifiedRotationSpeed.in(Units.RadiansPerSecond),
                    t
                );
            }
        }
    }

    public static final class VisionConstants {
        public static record PhotonCameraConfiguration(String cameraName, Transform3d robotToCamera) {}

        public static final PhotonCameraConfiguration[] kCameras = new PhotonCameraConfiguration[] {
            new PhotonCameraConfiguration("Camera1", new Transform3d()),
            new PhotonCameraConfiguration("Camera2", new Transform3d()),
            new PhotonCameraConfiguration("Camera3", new Transform3d())
        };

        public static final String kLogPath = "Subsystems/Vision";

        // TODO: update this
        public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFieldLayout.loadField(
            AprilTagFields.kDefaultField
        );;
    }

    public static final class ElevatorConstants {
        public static final String kLogPath = "Subsystems/Elevator";

        public static final int kMotorPort = 20;
        public static final boolean kInvertMotor = true;
        public static final int kFollowerPort = 21;

        public static final boolean kUseFOC = true;

        public static final double kSupplyCurrentLimit = 40.0;
        public static final double kStatorCurrentLimit = 1.75 * kSupplyCurrentLimit;

        public static final double kGearRatio = 20;
        public static final double kSprocketRadiusMeters = Conversions.inchesToMeters(0.819);

        public static final double kElevatorMaxPositionMeters = Conversions.inchesToMeters(69.736220);

        public static final double kUpLimitMotorRotations = Conversions.elevatorMetersToElevatorRotations(
            kElevatorMaxPositionMeters
        );
        public static final double kDownLimitMotorRotations = 0.0;

        public static final PIDConstants kPID = new PIDConstants(20, 0.0, 0.0);
        public static final FeedforwardConstants kFeedForward = new FeedforwardConstants(
            20.0,
            0.0,
            0.0,
            0.06
        );

        /** Distance from the top face of the bottom bar of the second stage to the floor */
        public static final double kElevatorBaseHeightMeters = SwerveModuleConstants.kWheelRadiusMeters
            + Conversions.inchesToMeters(2);
        /**
         * Distance from the center of the leading edge of the coral to the floor when the elevator is at its starting
         * configuration
         */
        public static final double kEndEffectorHeightMeters = kElevatorBaseHeightMeters
            + Conversions.inchesToMeters(25)
            + FieldConstants.CoralConstants.kCoralOuterRadiusMeters;

        public static final double kPositionToleranceMeters = Conversions.inchesToMeters(1);

        // The distance from the center of the reef that the elevator will be allowed to autonomously extend
        public static final double kAutoElevatorExtendRequiredDistanceMeters = 3.0;
    }

    public static final class EndEffectorConstants {
        // The maximum distance the chassis can be from the auto score location to score
        public static final double kAutoScoreMaxDistMeters = 0.02;
        public static final double kAutoScoreMaxAngleRadians = Conversions.degreesToRadians(5);
        public static final double kAutoScoreMaxLinearSpeedMetersPerSecond = 0.1;
        public static final double kAutoScoreMaxAngularSpeedRadiansPerSecond = Conversions.degreesToRadians(10);

        public static final double kScoreTimeoutSeconds = 2;

        public static final double kScoreVoltage = 6;
    }
}
