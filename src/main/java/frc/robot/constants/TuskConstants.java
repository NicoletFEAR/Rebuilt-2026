package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.GeneralConstants.DrivebaseMotorIds;
import frc.robot.controllers.Axis;
import frc.robot.controllers.ControllerType;
import frc.robot.util.SwerveModuleConstants;

public class TuskConstants {
    public static class TuskGeneralConstants {
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

    public static class TuskOperatorConstants {
        public static final ControllerType kDriveControllerType = ControllerType.PS5;
        public static final ControllerType kOperatorControllerType = ControllerType.PS5;

        public static final Axis kDriverAxis = new Axis(kDriveControllerType);
        public static final int kThrottleAxis = kDriverAxis.getAxis("kLeftY");
        public static final int kStrafeAxis = kDriverAxis.getAxis("kLeftX");
        public static final int kSteerAxis = kDriverAxis.getAxis("kRightX");
    }

    public static class TuskDeviceIds {
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

    public static class TuskDriveConstants implements SwerveConstantsInterface{
        public int getIsBatteryInBack() {
            return TuskGeneralConstants.isBatteryInBack();
        }

        public boolean getHasCanivore() {
            return TuskGeneralConstants.kHasCanivore;
        }

        public boolean getUsesDriveKrakens() {
            return true;
        }
        
        public double getSwerveDeadband() {
            return 0.75d;
        }

        public double getMaxModuleSpeed() {
            return 4.8d;
        }

        public double getMaxRotationsPerSecond() {
            return 1.5d;
        }

        public double getTrackWidth() {
            return 20.753888d;
        }
        
        public double getWheelBase() {
            return 20.753888d;
        }
        
        public double getDrivebaseRadius() {
            return Math.hypot(getTrackWidth(), getWheelBase());
        }

        public double getDriveGearRatio() {
            return 6.75d;
        }

        public double getTurnGearRatio() {
            return 12.8d;
        }
        
        public double getWheelDiameter() {
            return Meters.convertFrom(4, Inches);
        }

        public double getDriveRevToMeters() {
            return getDriveGearRatio() / (Math.PI * getWheelDiameter());
        }

        public double getTurnRotationsToDegrees() {
            return 360.0d / getTurnGearRatio();
        }

        public double getWheelCof() {
            return 1.0d;
        }

        public double getCurrentLimit() {
            return 103.0d;
        }

        public double getDriveKP() {
            return 2.7141d;
        }

        public double getDriveKI() {
            return 0.0d;
        }

        public double getDriveKD() {
            return 0.0d;
        }

        public double getDriveKS() {
            return 0.067703d;
        }

        public double getDriveKV() {
            return 2.4746d;
        }

        public double getDriveKA() {
            return 0.36888d;
        }
        
        public double getRampRate() {
            return 0.1d;
        }

        public double getTurnKP() {
            return 0.002d;
        }

        public double getTurnKI() {
            return 0.0d;
        }

        public double getTurnKD() {
            return 0.01d;
        }

        public double getTurnKS() {
            return 0.0d;
        }

        public double getTurnKV() {
            return 0.0d;
        }

        public double getTurnKA() {
            return 0.0d;
        }

        public double getFrontLeftOffset() {
            return -0.893798828125d;
        }

        public double getFrontRightOffset() {
            return -0.101806640625d;
        }

        public double getBackLeftOffset() {
            return -0.6767578125d;
        }

        public double getBackRightOffset() {
            return -0.470947265625d;
        }

        public double getAutoTargetKP() {
            return 2.0d;
        }

        public double getAutoTargetKI() {
            return 0.0d;
        }

        public double getAutoTargetKD() {
            return 0.0d;
        }
        
        public Translation2d[] getModuleTranslations() {
            Translation2d[] moduleTranslations = { 
                new Translation2d(getWheelBase()  / 2, getTrackWidth()  / 2),
                new Translation2d(getWheelBase()  / 2, -getTrackWidth() / 2),
                new Translation2d(-getWheelBase() / 2, getTrackWidth()  / 2),
                new Translation2d(-getWheelBase() / 2, -getTrackWidth() / 2)
            };

            return moduleTranslations;
        }

        public RobotConfig getRobotConfig() {
            return new RobotConfig(Units.lbsToKilograms(TuskGeneralConstants.kWeight),
                TuskGeneralConstants.kMoi,
                new ModuleConfig(getWheelDiameter() / 2,
                    getMaxModuleSpeed(),
                    getWheelCof(),
                    getUsesDriveKrakens() ?
                        DCMotor.getKrakenX60(1).withReduction(getDriveGearRatio()) :
                        DCMotor.getNEO(1).withReduction(getDriveGearRatio()),
                    getCurrentLimit(),
                    1),
                getModuleTranslations()
            );
        }

        public SwerveModuleConstants getFrontLeft() {
            return new SwerveModuleConstants(
                DrivebaseMotorIds.kFrontLeftDriveMotorId,
                DrivebaseMotorIds.kFrontLeftSteerEncoderId,
                DrivebaseMotorIds.kFrontLeftSteerEncoderId,
                getFrontLeftOffset()
            );
        }
        
        public SwerveModuleConstants getFrontRight() {
            return new SwerveModuleConstants(
                DrivebaseMotorIds.kFrontRightDriveMotorId,
                DrivebaseMotorIds.kFrontRightSteerMotorId,
                DrivebaseMotorIds.kFrontRightSteerEncoderId,
                getFrontRightOffset()
            );
        }

        public SwerveModuleConstants getBackLeft() {
            return new SwerveModuleConstants(
                DrivebaseMotorIds.kBackLeftDriveMotorId,
                DrivebaseMotorIds.kBackLeftSteerMotorId,
                DrivebaseMotorIds.kBackLeftSteerEncoderId,
                getBackLeftOffset()
            );
        }

        public SwerveModuleConstants getBackRight() {
            return new SwerveModuleConstants(
                DrivebaseMotorIds.kBackRightDriveMotorId,
                DrivebaseMotorIds.kBackRightSteerMotorId,
                DrivebaseMotorIds.kBackRightSteerEncoderId,
                getBackRightOffset()
            );
        }
    }

    public class LauncherConstants {
        public static double[][] kAutoAimHoodPositions = {
            {1.85d, 0.0d},
            {4.45d, 0.6315d}
        };

        public static double[][] kAutoAimSpeeds = {
            {1.85d, 0.7d},
            {4.45d, 1.0d}
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
        public static final double kIndexerGearRatio = 1.0d;

        //TODO: Tune PID
        public static final double kIndexerKP = 1.0d;
        public static final double kIndexerKI = 0.0d;
        public static final double kIndexerKD = 0.0d;

        public static final double kIndexerOffVoltage = 0.0d;
        public static final double kIndexerIndexVoltage = 0.0d;
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
        public static final double kTargetAmountConstant = 2.0d;
        public static final double kSpeedsConstant = 1.0d;
        public static final double kRotationsConstant = 1.5d;
        public static final double kDistanceConstant = 0.25d;
        public static final double kAreaConstant = 0.01d;

        public static final double kOffsetTolerance = 1000.0d;
    }
}
