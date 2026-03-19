package frc.robot.constants;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.GeneralConstants.DrivebaseMotorIds;
import frc.robot.controllers.Axis;
import frc.robot.controllers.ControllerType;
import frc.robot.util.SwerveModuleConstants;

public class HadesConstants {
    public static class HadesGeneralConstants {
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

    public static class HadesOperatorConstants {
        public static final ControllerType kDriveControllerType = ControllerType.PS5;

        public static final Axis kDriverAxis = new Axis(kDriveControllerType);
        public static final int kThrottleAxis = kDriverAxis.getAxis("kLeftY");
        public static final int kStrafeAxis = kDriverAxis.getAxis("kLeftX");
        public static final int kSteerAxis = kDriverAxis.getAxis("kRightX");
    }

    public static class HadesDriveConstants implements SwerveConstantsInterface {
        public int getIsBatteryInBack() {
            return HadesGeneralConstants.isBatteryInBack();
        }

        public boolean getHasCanivore() {
            return HadesGeneralConstants.kHasCanivore;
        }

        public boolean getUsesDriveKrakens() {
            return false;
        }

        public double getSwerveDeadband() {
            return 0.75d;
        }

        public double getMaxModuleSpeed() {
            return MetersPerSecond.convertFrom(16.18, FeetPerSecond);
        }

        public double getMaxRotationsPerSecond() {
            return 1.5d;
        }

        // Distance between centers of right and left wheels on robot
        public double getTrackWidth() {
            return 20.75d;
        }

        // Distance between centers of front and back wheels on robot
        public double getWheelBase() {
            return 20.75d;
        }

        // Defined as half the diagonal of the drivebase
        public double getDrivebaseRadius() {
            return Math.hypot(getTrackWidth(), getWheelBase());
        }
        
        public double getDriveGearRatio() {
            return 6.75d;
        }

        public double getTurnGearRatio() {
            return 12.8d;
        }
        
        // TODO: Run the WheelCharacterization command to find the wheel diameter
        public double getWheelDiameter() {
            return Meters.convertFrom(3.95552051507874003, Inches);
        }

        public double getDriveRevToMeters() {
            return getDriveGearRatio() / (Math.PI * getWheelDiameter());
        }

        public double getTurnRotationsToDegrees() {
            return 360.0 / getTurnGearRatio();
        }

        public double getWheelCof() {
            return 1.0d;
        }

        public double getCurrentLimit() {
            return 103.0d;
        }

        // TODO: Tune PID
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
            return 1.0d;
        }

        public double getTurnKP() {
            return 0.02d;
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
            return -0.403076171875d;
        }

        public double getFrontRightOffset() {
            return -0.0966796875d;
        }

        public double getBackLeftOffset() {
            return -0.109375d;
        }

        public double getBackRightOffset() {
            return -0.310546875d;
        }

        public double getAutoTargetKP() {
            return 2.0d;
        }

        public double getAutoTargetKI() {
            return 0.0d;
        }

        public double getAutoTargetKD() {
            return 0.2d;
        }

        // Positions of all the swerve modules relative to the center of the drivebase
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
            return new RobotConfig(Units.lbsToKilograms(HadesGeneralConstants.kWeight),
                HadesGeneralConstants.kMoi,
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
}
