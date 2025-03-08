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
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.SwerveConstants.SwerveModuleConstants;
import frc.robot.util.Conversions;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.FieldConstants.ReefConstants.ReefHeight;

public final class Constants {
    public static final Mode kCurrentMode = Mode.REAL;

    public static final Alliance kDefaultAlliance = Alliance.Blue;

    public static final double kLoopPeriodSecs = Robot.defaultPeriodSecs;

    public static final int kPDPPort = 1;

    public static final double kJoystickDeadzone = 0.15;

    public static final double kBumperWidthMeters = Conversions.inchesToMeters(3.25);
    public static final double kChassisLengthMeters = Conversions.inchesToMeters(29);

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

            public static final int kFrontLeftDrivePort = 10;
            public static final int kFrontLeftTurnPort = 20;
            public static final int kFrontLeftEncoderPort = 30;
            public static final int kFrontRightDrivePort = 11;
            public static final int kFrontRightTurnPort = 21;
            public static final int kFrontRightEncoderPort = 31;
            public static final int kBackLeftDrivePort = 13;
            public static final int kBackLeftTurnPort = 23;
            public static final int kBackLeftEncoderPort = 33;
            public static final int kBackRightDrivePort = 12;
            public static final int kBackRightTurnPort = 22;
            public static final int kBackRightEncoderPort = 32;

            public static final Translation2d kModulePosFL = new Translation2d(0.283989, 0.283989);
            public static final Translation2d kModulePosFR = new Translation2d(0.283989, -0.283989);
            public static final Translation2d kModulePosBL = new Translation2d(-0.283989, 0.283989);
            public static final Translation2d kModulePosBR = new Translation2d(-0.283989, -0.283989);

            // Units in module space not motor space
            public static final PIDConstants kDrivePID = new PIDConstants(
                0.5, // Volts per m/s error
                0,
                0
            );

            public static final FeedforwardConstants kDriveFeedforward = new FeedforwardConstants(0.14, 6);

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

            public static final FeedforwardConstants kTurnFeedforward = new FeedforwardConstants(0, 0);

            public static final int kDriveStatorCurrentLimit = 92;
            public static final int kDriveSupplyCurrentLowerLimit = 40;
            public static final int kDriveSupplyCurrentLimit = 60;

            public static final int kTurnSupplyCurrentLimit = 30;

            public static final LinearVelocity kDriveMaxFreeSpeed = Units.FeetPerSecond.of(12.9);
        }

        public static final int kPigeonPort = 40;

        public static final boolean kTeleopFieldRelative = true;

        public static final Pose2d kStartingPose = new Pose2d();
        public static final String kLogPath = "Subsystems/Swerve";
        public static final double kFrameWidth = Conversions.inchesToMeters(29);
        public static final double kFrameHeight = Conversions.inchesToMeters(29);
        public static final double kDriveBaseRadiusMeters = SwerveModuleConstants.kModulePosFL.getDistance(
            Translation2d.kZero
        );

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
            47.17360648,
            4.2659057746, // (1 / 12) * mass * (length^2 + width^2)
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
                SwerveModuleConstants.kDriveSupplyCurrentLowerLimit,
                1
            ),
            SwerveModuleConstants.kModulePosFL,
            SwerveModuleConstants.kModulePosFR,
            SwerveModuleConstants.kModulePosBL,
            SwerveModuleConstants.kModulePosBR
        );

        public static final PathConstraints kPPPathFindConstraints = new PathConstraints(
            Units.MetersPerSecond.of(2.5),
            Units.MetersPerSecondPerSecond.of(3),
            Units.RotationsPerSecond.of(1.5),
            Units.RotationsPerSecondPerSecond.of(3)
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
            Units.RotationsPerSecond.of(2),
            Units.RotationsPerSecond.of(1),
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
            new PhotonCameraConfiguration(
                "FrontCameraL",
                new Transform3d(
                    0.27,
                    0.15,
                    SwerveModuleConstants.kWheelRadiusMeters + (65.0 / 1000.0) + (7.037 / 1000.0),
                    new Rotation3d(0.0, Conversions.degreesToRadians(5), Conversions.degreesToRadians(5))
                )
            ),
            new PhotonCameraConfiguration(
                "FrontCameraR",
                new Transform3d(
                    0.27,
                    -0.15,
                    SwerveModuleConstants.kWheelRadiusMeters + (65.0 / 1000.0) + (7.037 / 1000.0),
                    new Rotation3d(0.0, Conversions.degreesToRadians(5), Conversions.degreesToRadians(-5))
                )
            ),
            new PhotonCameraConfiguration(
                "RearCamera",
                new Transform3d(
                    -0.27,
                    0.0,
                    SwerveModuleConstants.kWheelRadiusMeters + (65.0 / 1000.0) + (7.037 / 1000.0),
                    new Rotation3d(0.0, Conversions.degreesToRadians(25), Conversions.degreesToRadians(220))
                )
            )
        };

        public static final String kLogPath = "Subsystems/Vision";

        public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFieldLayout.loadField(
            AprilTagFields.k2025ReefscapeWelded // Peachtree uses Welded
        );
    }

    public static final class ElevatorConstants {
        public static final String kLogPath = "Subsystems/Elevator";

        public static final int kMotorPort = 40;
        public static final boolean kInvertMotor = true;
        public static final int kFollowerPort = 41;

        public static final boolean kUseMotorPID = false;

        public static final boolean kUseFOC = true;

        public static final double kSupplyCurrentLimit = 40.0;
        public static final double kStatorCurrentLimit = 1.75 * kSupplyCurrentLimit;

        public static final double kGearRatio = 12;
        public static final double kSprocketRadiusMeters = Conversions.inchesToMeters(0.819);

        public static final double kElevatorMaxPositionMeters = Conversions.inchesToMeters(69.736220);

        public static final double kForwardSoftLimitRad = 29.6;
        public static final double kReverseSoftLimitRad = 0.1;

        public static final PIDConstants kPID = new PIDConstants(
            0.0,
            // 0.13, // V / rad
            0.0,
            0.0
        );
        public static final FeedforwardConstants kFeedForward = new FeedforwardConstants(
            0.09,
            0.244,
            0.0,
            0.40
            // 0.0860202312 // V
        );
        //        public static final double kElevatorCruiseVelocityRadPerSec = Conversions.elevatorMetersToElevatorRadians(0.75);
        //        public static final double kElevatorAccelerationRadPerSecPerSec = Conversions.elevatorMetersToElevatorRadians(
        //            2
        //        )s
        public static final double kElevatorCruiseVelocityRadPerSec = Conversions.elevatorMetersToElevatorRadians(1.2);
        public static final double kElevatorAccelerationRadPerSecPerSec = Conversions.elevatorMetersToElevatorRadians(
            3
        );

        /** Distance between belly pan and elevator base */
        public static final double kBellyPanToElevatorBaseMeters = Conversions.inchesToMeters(3);

        /** Distance from the top face of the bottom bar of the second stage to the floor */
        public static final double kElevatorBaseHeightMeters = SwerveModuleConstants.kWheelRadiusMeters
            + kBellyPanToElevatorBaseMeters;

        // TODO: Verify this
        /**
         * Distance from the center of the leading edge of the coral to the floor when the elevator is at its starting
         * configuration
         */
        public static final double kEndEffectorHeightMeters = Conversions.inchesToMeters(21.5);

        public static final double kPositionToleranceMeters = Conversions.inchesToMeters(0.5);

        // The distance from the center of the reef that the elevator will be allowed to autonomously extend
        public static final double kAutoElevatorExtendRequiredDistanceMeters = 4.5;

        public static final double kL1HeightMeters = 0.0;
        public static final double kL2HeightMeters = ReefHeight.L2.getHeight() - Conversions.inchesToMeters(2);
        public static final double kL3HeightMeters = ReefHeight.L3.getHeight() - Conversions.inchesToMeters(2);
        public static final double kL4HeightMeters = ReefHeight.L4.getHeight() - Conversions.inchesToMeters(2);
    }

    public static final class EndEffectorConstants {
        public static final String kLogPath = "Subsystems/EndEffector";

        /**
         * Length of end effector measured from bottom of the poly carb to the top edge of the bottom plane that the
         * coral rests on
         */
        public static final double kEndEffectorLengthMeters = Conversions.inchesToMeters(11.967);

        public static final int kLeftPort = 51;
        public static final int kRightPort = 50;

        public static final int kCoralSensorPort = 1;
        public static final int kCoralSensorLeadingPort = 0;

        public static final double kSupplyCurrentLimit = 40;
        public static final double kStatorCurrentLimit = 1.75 * kSupplyCurrentLimit;

        public static final double kGearRatio = 12;

        public static final boolean kInvertMotor = true;

        // The maximum distance the chassis can be from the auto score location to score
        public static final double kAutoScoreMaxDistMeters = 0.02;
        public static final double kAutoScoreMaxAngleRadians = Conversions.degreesToRadians(5);
        public static final double kAutoScoreMaxLinearSpeedMetersPerSecond = 0.1;
        public static final double kAutoScoreMaxAngularSpeedRadiansPerSecond = Conversions.degreesToRadians(10);

        // Angle of coral measured from the horizontal
        public static final Rotation2d kAngle = Rotation2d.fromDegrees(35);

        public static final double kScoreTimeoutSeconds = 2;

        public static final double kScoreVoltage = 10;
        public static final double kIntakeVoltage = 6;
        public static final double kIntakeSlowVoltage = 5;
        public static final double kIntakeSlowSlowVoltage = -1.75;
    }

    public static final class BallsIntakeConstants {
        public static final double kScorePower = 0.5;
        public static final int kMotorPort = 69; //MAKE SURE TO CHANGE THIS!!
    }

    public static final class LEDConstants {
        public static final int kCANdlePort = 60;
        public static final int kLEDCount = 10;
    }

    public static final class ClimbConstants {
        public static final String kLogPath = "Subsystems/Climb";

        public static final double kGearRatio = 125.0;

        public static final double kSupplyCurrentLimit = 40.0;

        public static final boolean kInvertMotor = false;

        public static final Rotation2d kStartingPosition = Rotation2d.fromDegrees(90);
        public static final Rotation2d kMaxPosition = Rotation2d.fromDegrees(100);
        public static final Rotation2d kMinPosition = Rotation2d.fromDegrees(-10);
        public static final Rotation2d kClimbPosition = Rotation2d.fromDegrees(10);
        public static final Rotation2d kCagePosition = Rotation2d.fromDegrees(95);

        public static final double kClimbingVoltage = -5.0;
        public static final double kHoldingVoltage = -0.0;

        public static final int kMotorPort = 60;

        public static final PIDConstants kPID = new PIDConstants(0.0, 0.0, 0.0);

        public static final double kPositionToleranceRad = Conversions.degreesToRadians(2);
    }
}
