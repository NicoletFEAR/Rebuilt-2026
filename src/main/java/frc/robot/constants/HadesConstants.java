package frc.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.DriveConstants;
import frc.robot.controllers.Axis;
import frc.robot.controllers.ControllerType;
import frc.robot.util.SwerveModuleConstants;

public class HadesConstants {
    public class HadesGeneralConstants {
        public static final double moi = 6.883d;
        public static final double weight = 115;

        public static final boolean hasCanivore = false;
        public static final boolean batteryBack = false;
    }

    public class HadesOperatorConstants {
        public static final ControllerType operatorControllerType = ControllerType.XBOX;
        public static final ControllerType driveControllerType = ControllerType.PS5;

        public static final Axis operatorAxis = new Axis(operatorControllerType);
        public static final int kThrottleAxis = operatorAxis.getAxis("kLeftY");
        public static final int kStrafeAxis = operatorAxis.getAxis("kLeftX");
        public static final int kSteerAxis = operatorAxis.getAxis("kRightX");
    }

    public class HadesDriveConstants {
        public static final boolean usesDriveKrakens = false;

        public static final double maxModuleSpeed = 16.18d;
        public static final double getMaxRotationsPerSecond = 1.5d;
        public static final double trackWidth = 20.75d;
        public static final double wheelBase = 20.75d;
        public static final double driveGearRatio = 6.746031746031747d;
        public static final double turnGearRatio = 21.428571428571427d;
        public static final double wheelDiameter = 0.100470221083d;
        public static final double driveRevToMeters = driveGearRatio / (Math.PI * wheelDiameter);
        public static final double turnRotationsToDegrees = 360.0 / turnGearRatio;
        public static final double wheelCof = 1.0d;
        public static final double currentLimit = 103.0d;

        public static final double driveKP = 2.7141d;
        public static final double driveKI = 0.0d;
        public static final double driveKD = 0.0d;
        public static final double driveKS = 0.0d;
        public static final double driveKV = 0.0d;
        public static final double driveKA = 0.0d;

        public static final double rampRate = 0.1d;

        public static final double turnKP = 0.02d;
        public static final double turnKI = 0.0d;
        public static final double turnKD = 0.01d;
        public static final double turnKS = 0.0d;
        public static final double turnKV = 0.0d;
        public static final double turnKA = 0.0d;

        public static final double rotationTolerance = 0.01d;

        public static final double frontLeftOffset = -0.403076171875d;
        public static final double frontRightOffset = -0.0966796875d;
        public static final double backLeftOffset = -0.109375d;
        public static final double backRightOffset = -0.310546875d;

        public static final double getAutoTargetKP = 2.0d;
        public static final double getAutoTargetKi = 0.0d;
        public static final double getAutoTargetKd = 0.2d;

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

        public static final RobotConfig kRobotConfig = new RobotConfig(Units.lbsToKilograms(HadesGeneralConstants.weight),
            HadesGeneralConstants.moi,
            new ModuleConfig(wheelDiameter / 2,
                maxModuleSpeed,
                wheelCof,
                DriveConstants.usesDriveKrakens() ?
                    DCMotor.getKrakenX60(1).withReduction(driveGearRatio) :
                    DCMotor.getNEO(1).withReduction(driveGearRatio),
                currentLimit,
                1),
            DriveConstants.kModuleTranslations
        );

        public static final SwerveModuleConstants kFrontLeft = new SwerveModuleConstants(
            DeviceIds.getFrontLeftDriveId(),
            DeviceIds.getFrontLeftSteerId(),
            DeviceIds.getFrontLeftSteerEncoderId(),
            frontLeftOffset
        );

        public static final SwerveModuleConstants kFrontRight = new SwerveModuleConstants(
            DeviceIds.getFrontRightDriveId(),
            DeviceIds.getFrontRightSteerId(),
            DeviceIds.getFrontRightSteerEncoderId(),
            frontRightOffset
        );

        public static final SwerveModuleConstants kBackLeft = new SwerveModuleConstants(
            DeviceIds.getBackLeftDriveId(),
            DeviceIds.getBackLeftSteerId(),
            DeviceIds.getBackLeftSteerEncoderId(),
            backLeftOffset
        );

        public static final SwerveModuleConstants kBackRight = new SwerveModuleConstants(
            DeviceIds.getBackRightDriveId(),
            DeviceIds.getBackRightSteerId(),
            DeviceIds.getBackRightSteerEncoderId(),
            backRightOffset
        );

        public static final Translation2d kHubPosition = new Translation2d(4.619, 4.033);
    }
}
