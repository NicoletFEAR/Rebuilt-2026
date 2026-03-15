package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.GeneralConstants.DrivebaseMotorIds;
import frc.robot.controllers.Axis;
import frc.robot.controllers.ControllerType;
import frc.robot.util.SwerveModuleConstants;

public class TuskConstants {
    public class TuskGeneralConstants {
        public static final double kMoi = 6.883d;
        //TODO: Get the actual Robot Weight
        public static final double kWeight = 115;

        public static final boolean kHasCanivore = true;
        public static final boolean kBatteryBack = true;

        public static int isBatteryInBack() {
            if (kBatteryBack == true) {
                return -1;
            } else {
                return 1;
            }
        }
    }

    public class TuskOperatorConstants {
        public static final ControllerType kDriveControllerType = ControllerType.PS5;
        public static final ControllerType kOperatorControllerType = ControllerType.PS5;

        public static final Axis kDriverAxis = new Axis(kDriveControllerType);
        public static final int kThrottleAxis = kDriverAxis.getAxis("kLeftY");
        public static final int kStrafeAxis = kDriverAxis.getAxis("kLeftX");
        public static final int kSteerAxis = kDriverAxis.getAxis("kRightX");
    }

    public class TuskDeviceIds {
        public static final int kLeftLauncherId = 14;
        public static final int kRightLauncherId = 15;
        public static final int kHoodId = 23;
        public static final int kHoodEncoderId = 22;
        public static final int kIndexerId = 16;
        public static final int kIntakeDriverId = 18;
        public static final int kIntakePivotId = 19;
        public static final int kClimbId = 20;
        public static final int kLEDId = 21;
    }

    public class TuskDriveConstants {
        public static final boolean kUsesDriveKrakens = true;
        public static final double kSwerveDeadband = 0.75d;

        public static final double maxModuleSpeed = 4.8d;
        public static final double getMaxRotationsPerSecond = 1.5d;
        // Distance between centers of right and left wheels on robot
        public static final double trackWidth = 20.753888d;
        // Distance between centers of front and back wheels on robo
        public static final double wheelBase = 20.753888d;
        // Defined as half the diagonal of the drivebase
        public static final double getDrivebaseRadius = Math.hypot(trackWidth, wheelBase);
        public static final double driveGearRatio = 6.75d;
        public static final double turnGearRatio = 12.8;
        // TODO: Run the WheelCharacterization command to find the wheel diameter
        public static final double wheelDiameter = Meters.convertFrom(4, Inches);
        public static final double driveRevToMeters = driveGearRatio / (Math.PI * wheelDiameter);
        public static final double turnRotationsToDegrees = 360.0 / turnGearRatio;
        public static final double wheelCof = 1.0d;
        public static final double currentLimit = 103.0d;

        // TODO: Tune PID
        public static final double driveKP = 2.7141d;
        public static final double driveKI = 0.0d;
        public static final double driveKD = 0.0d;
        public static final double driveKS = 0.067703d;
        public static final double driveKV = 2.4746d;
        public static final double driveKA = 0.36888d;

        public static final double rampRate = 0.1d;

        public static final double turnKP = 0.02d;
        public static final double turnKI = 0.0d;
        public static final double turnKD = 0.01d;
        public static final double turnKS = 0.0d;
        public static final double turnKV = 0.0d;
        public static final double turnKA = 0.0d;

        public static final double rotationTolerance = 0.01d;

        public static final double frontLeftOffset = -0.893798828125d;
        public static final double frontRightOffset = -0.101806640625d;
        public static final double backLeftOffset = -0.6767578125d;
        public static final double backRightOffset = -0.470947265625d;

        public static final double getAutoTargetKP = 2.0d;
        public static final double getAutoTargetKI = 0.0d;
        public static final double getAutoTargetKD = 0.2d;

        // Positions of all the swerve modules relative to the center of the drivebase
        public static final Translation2d[] kModuleTranslations = {
            new Translation2d(wheelBase  / 2, trackWidth  / 2),
            new Translation2d(wheelBase  / 2, -trackWidth / 2),
            new Translation2d(-wheelBase / 2, trackWidth  / 2),
            new Translation2d(-wheelBase / 2, -trackWidth / 2)
        };

        public static final SwerveModuleState[] kXWheels = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-135))
        };

        public static final RobotConfig kRobotConfig = new RobotConfig(Units.lbsToKilograms(TuskGeneralConstants.kWeight),
            TuskGeneralConstants.kMoi,
            new ModuleConfig(wheelDiameter / 2,
                maxModuleSpeed,
                wheelCof,
                TuskDriveConstants.kUsesDriveKrakens ?
                    DCMotor.getKrakenX60(1).withReduction(driveGearRatio) :
                    DCMotor.getNEO(1).withReduction(driveGearRatio),
                currentLimit,
                1),
            TuskDriveConstants.kModuleTranslations
        );

        public static final SwerveModuleConstants kFrontLeft = new SwerveModuleConstants(
            DrivebaseMotorIds.kFrontLeftDriveMotorId,
            DrivebaseMotorIds.kFrontLeftSteerEncoderId,
            DrivebaseMotorIds.kFrontLeftSteerEncoderId,
            frontLeftOffset
        );

        public static final SwerveModuleConstants kFrontRight = new SwerveModuleConstants(
            DrivebaseMotorIds.kFrontRightDriveMotorId,
            DrivebaseMotorIds.kFrontRightSteerMotorId,
            DrivebaseMotorIds.kFrontRightSteerEncoderId,
            frontRightOffset
        );

        public static final SwerveModuleConstants kBackLeft = new SwerveModuleConstants(
            DrivebaseMotorIds.kBackLeftDriveMotorId,
            DrivebaseMotorIds.kBackLeftSteerMotorId,
            DrivebaseMotorIds.kBackLeftSteerEncoderId,
            backLeftOffset
        );

        public static final SwerveModuleConstants kBackRight = new SwerveModuleConstants(
            DrivebaseMotorIds.kBackRightDriveMotorId,
            DrivebaseMotorIds.kBackRightSteerMotorId,
            DrivebaseMotorIds.kBackRightSteerEncoderId,
            backRightOffset
        );

        public static final Translation2d kHubPosition = new Translation2d(4.619, 4.033);
    }

    public class LauncherConstants {
        public static double[][] kAutoAimHoodPositions = {
            {1.85, 0.0},
            {4.45, .6315}
        };

        public static double[][] kAutoAimSpeeds = {
            {1.85, 0.7},
            {4.45, 1.0}
        };

        public static final double kLauncherGearRatio = 4.0d/5.0d;
        public static final double kLauncherHoodGearRatio = 30.0d;

        public static final double kHoodKP = 5.0d;
        public static final double kHoodKI = 0.0d;
        public static final double kHoodKD = 0.0d;

        public static final double kHoodOffset = -0.6044921875d;
        public static final double kHoodSetpointTolerance = 0.01d;
        public static final double kHoodManualModifier = 0.0125d;
        public static final double kHoodHomePosition = 0.0d;
        public static final double kHoodMinPosition = 0.0d;
        public static final double kHoodMaxPosition = 0.6315d;

        //TODO: Tune PID
        public static final double kLauncherKP = 1.0d;
        public static final double kLauncherKI = 0.0d;
        public static final double kLauncherKD = 0.0d;

        public static final double kLauncherVelocityTolerance = 5.0d;
        public static final double kLauncherLaunchVelocity = 110.0d;
        public static final double kLauncherOffVoltage = 0.0d;
        public static final double kLauncherLaunchVoltage = 12.0d;
    }

    public class IndexerConstants {
        public static final double kIndexerGearRatio = 1.0;

        //TODO: Tune PID
        public static final double kIndexerKP = 1.0;
        public static final double kIndexerKI = 0.0;
        public static final double kIndexerKD = 0.0;

        public static final double kIndexerOffVoltage = 0.0;
        public static final double kIndexerIndexVoltage = 0.0;
    }

    public class IntakeConstants {
        public static final double kIntakeDriverGearRatio = 3.0d/5.0d;
        public static final double kIntakePivotGearRatio = 80.0d/3.0d;

        //TODO: Tune PID
        public static final double kIntakeDriverKP = 1.0d;
        public static final double kIntakeDriverKI = 0.0d;
        public static final double kIntakeDriverKD = 0.0d;

        public static final double kIntakePivotKP = 25.0d;
        public static final double kIntakePivotKI = 0.0d;
        public static final double kIntakePivotKD = 0.0d;

        public static final double kIntakeDriverOffVoltage = 0.0d;
        public static final double kIntakeDriverIntakeVoltage = 12.0d;
        public static final double kIsStuckOnBallThreshold = 23.0d;

        public static final double kSetpointTolerance = 0.025d;
        public static final double kManualModifier = 0.005d;
        public static final double kMinPosition = 0.0d;
        public static final double kMaxPosition = 0.099609d;
        public static final double kHomePosition = 0.0d;
        public static final double kJostlePosition = 0.0473635d;
        public static final double kHoldPosition = 0.094727d;
        public static final double kOutPosition = 0.099609d;
    }

    public class LEDConstants {
        public static final int kSlotStart = 8;
        public static final int kSlotEnd = 84;
    }

    public final class VisionConstants {
        public static final double kTargetAmountConstant = 2.0;
        public static final double kSpeedsConstant = 1.0;
        public static final double kRotationsConstant = 1.5;
        public static final double kDistanceConstant = 0.25;
        public static final double kAreaConstant = 0.01;

        public static final double kOffsetTolerance = 1000.0;
    }
}
