package frc.robot.constants;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

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

public class HadesConstants {
    public class HadesGeneralConstants {
        public static final double kMoi = 6.883d;
        public static final double kWeight = 115;

        public static final boolean kHasCanivore = false;
        public static final boolean kBatteryBack = false;

        public static int isBatteryInBack() {
            if (kBatteryBack == true) {
                return -1;
            } else {
                return 1;
            }
        }
    }

    public class HadesOperatorConstants {
        public static final ControllerType kDriveControllerType = ControllerType.PS5;

        public static final Axis kDriverAxis = new Axis(kDriveControllerType);
        public static final int kThrottleAxis = kDriverAxis.getAxis("kLeftY");
        public static final int kStrafeAxis = kDriverAxis.getAxis("kLeftX");
        public static final int kSteerAxis = kDriverAxis.getAxis("kRightX");
    }

    public class HadesDriveConstants {
        public static final boolean kUsesDriveKrakens = false;
        public static final double kSwerveDeadband = 0.75d;

        public static final double maxModuleSpeed = MetersPerSecond.convertFrom(16.18, FeetPerSecond);
        public static final double getMaxRotationsPerSecond = 1.5d;
        // Distance between centers of right and left wheels on robot
        public static final double trackWidth = 20.75d;
        // Distance between centers of front and back wheels on robo
        public static final double wheelBase = 20.75d;
        // Defined as half the diagonal of the drivebase
        public static final double getDrivebaseRadius = Math.hypot(trackWidth, wheelBase);
        public static final double driveGearRatio = 6.75d;
        public static final double turnGearRatio = 12.8;
        // TODO: Run the WheelCharacterization command to find the wheel diameter
        public static final double wheelDiameter = Meters.convertFrom(3.95552051507874003, Inches);
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

        public static final RobotConfig kRobotConfig = new RobotConfig(Units.lbsToKilograms(HadesGeneralConstants.kWeight),
            HadesGeneralConstants.kMoi,
            new ModuleConfig(wheelDiameter / 2,
                maxModuleSpeed,
                wheelCof,
                HadesDriveConstants.kUsesDriveKrakens ?
                    DCMotor.getKrakenX60(1).withReduction(driveGearRatio) :
                    DCMotor.getNEO(1).withReduction(driveGearRatio),
                currentLimit,
                1),
            HadesDriveConstants.kModuleTranslations
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
}
