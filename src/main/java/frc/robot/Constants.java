// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Properties;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS5Controller.Axis;
import frc.robot.util.SwerveModuleConstants;
import frc.robot.util.Utils;

public final class Constants {
    private static Properties m_properties;

    public static void instantiateProperties() {
        m_properties = new Properties();
        String robot_name = Utils.getMacAddress();

        try (InputStream input = new FileInputStream(new File(Filesystem.getDeployDirectory(), robot_name + ".properties"))) {
            m_properties.load(input);
        } catch (IOException ex) {
            DriverStation.reportError("Constants file not found", ex.getStackTrace());
        }
    }

    // How frequently the state of the robot updates during simulations
    // Set to 0.02 to represent once every loop (20 ms)
    public static final double kdt = 0.02;

    // These are the unique identifiers for each RoboRIO we plan to use
    public static final HashMap<String, String> kMacToName;
    static {
        kMacToName = new HashMap<>();
        kMacToName.put("00:80:2F:17:CD:DD", "kitbot");
        kMacToName.put("00:80:2F:17:B5:2A", "hades");
        kMacToName.put("00:80:2F:41:9F:8A", "competition");
    }

    public final class OperatorConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kThrottleAxis = Axis.kLeftY.value;
        public static final int kStrafeAxis = Axis.kLeftX.value;
        public static final int kSteerAxis = Axis.kRightX.value;

        public static final double kDefaultSpeed = 1;
        public static final double kSlowSpeed = 0.4;
    }

    public final class DriveConstants {
        public static double getSwerveDeadband() {
            return 0.075;
        }

        public static double getMaxModuleSpeed() {
            return Units.feetToMeters(Double.parseDouble(m_properties.getProperty("drive.max-module-speed")));
        }

        public static double getMaxRotationsPerSecond() {
            return Double.parseDouble(m_properties.getProperty("drive.max-rotations-per-second"));
        }

        // Distance between centers of right and left wheels on robot
        public static double getTrackWidth() {
            return Units.inchesToMeters(Double.parseDouble(m_properties.getProperty("drive.track-width")));
        }

        // Distance between centers of front and back wheels on robot
        public static double getWheelBase() {
            return Units.inchesToMeters(Double.parseDouble(m_properties.getProperty("drive.wheel-base")));
        }

        // Defined as half the diagonal of the drivebase
        public static double getDrivebaseRadius() {
            return Math.hypot(getTrackWidth(), getWheelBase());
        }

        // TODO: Remember to change if we modify the swerve modules
        public static double getDriveGearRatio() {
            return Double.parseDouble(m_properties.getProperty("drive.drive-gear-ratio"));
        }

        public static double getTurnGearRatio() {
            return Double.parseDouble(m_properties.getProperty("drive.turn-gear-ratio"));
        }

        // TODO: Run the WheelCharacterization command to find the wheel diameter
        public static double getWheelDiameter() {
            return Double.parseDouble(m_properties.getProperty("drive.wheel-diameter"));
        }

        public static double getDriveRevToMeters() {
            return getDriveGearRatio() / (Math.PI * getWheelDiameter());
        }

        public static double getTurnRotationsToDegrees() {
            return 360.0 / getTurnGearRatio();
        }

        public static double getWeight() {
            return Double.parseDouble(m_properties.getProperty("weight"));
        }

        // TODO: Calculate MOI based on the values given by Elijah from the CAD
        public static double getMOI() {
            return Double.parseDouble(m_properties.getProperty("moi"));
        }

        public static double getWheelCOF() {
            return Double.parseDouble(m_properties.getProperty("drive.wheel-cof"));
        }

        public static double getCurrentLimit() {
            return Double.parseDouble(m_properties.getProperty("drive.current-limit"));
        }

        // TODO: Tune PID
        public static double getDriveKP() {
            return Double.parseDouble(m_properties.getProperty("drive.drive-kp"));
        }

        public static double getDriveKI() {
            return Double.parseDouble(m_properties.getProperty("drive.drive-ki"));
        }

        public static double getDriveKD() {
            return Double.parseDouble(m_properties.getProperty("drive.drive-kd"));
        }

        public static double getDriveKS() {
            return Double.parseDouble(m_properties.getProperty("drive.drive-ks"));
        }

        public static double getDriveKV() {
            return Double.parseDouble(m_properties.getProperty("drive.drive-kv"));
        }

        public static double getDriveKA() {
            return Double.parseDouble(m_properties.getProperty("drive.drive-ka"));
        }

        public static double getDriveRampRate() {
            return Double.parseDouble(m_properties.getProperty("drive.drive-ramp-rate"));
        }

        public static double getTurnKP() {
            return Double.parseDouble(m_properties.getProperty("drive.turn-kp"));
        }

        public static double getTurnKI() {
            return Double.parseDouble(m_properties.getProperty("drive.turn-ki"));
        }

        public static double getTurnKD() {
            return Double.parseDouble(m_properties.getProperty("drive.turn-kd"));
        }

        public static double getTurnKFF() {
            return Double.parseDouble(m_properties.getProperty("drive.turn-kff"));
        }

        // TODO: Calibrate wheels
        public static double getFrontLeftOffset() {
            return Double.parseDouble(m_properties.getProperty("drive.front-left-offset"));
        }

        public static double getFrontRightOffset() {
            return Double.parseDouble(m_properties.getProperty("drive.front-right-offset"));
        }

        public static double getBackLeftOffset() {
            return Double.parseDouble(m_properties.getProperty("drive.back-left-offset"));
        }

        public static double getBackRightOffset() {
            return Double.parseDouble(m_properties.getProperty("drive.back-right-offset"));
        }

        // Positions of all the swerve modules relative to the center of the drivebase
        public static final Translation2d[] kModuleTranslations = {
            new Translation2d(getWheelBase()  / 2, getTrackWidth()  / 2),
            new Translation2d(getWheelBase()  / 2, -getTrackWidth() / 2),
            new Translation2d(-getWheelBase() / 2, getTrackWidth()  / 2),
            new Translation2d(-getWheelBase() / 2, -getTrackWidth() / 2)
        };

        public static final SwerveModuleState[] kXWheels = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-135))
        };

        public static final RobotConfig kRobotConfig = new RobotConfig(Units.lbsToKilograms(getWeight()),
            getMOI(),
            new ModuleConfig(getWheelDiameter() / 2,
                getMaxModuleSpeed(),
                getWheelCOF(),
                DCMotor.getKrakenX60(1).withReduction(getDriveGearRatio()),
                getCurrentLimit(),
                1),
            DriveConstants.kModuleTranslations
        );

        public static final int kPigeonId = 13;

        public static final int kFrontLeftDriveMotor = 2;
        public static final int kFrontLeftSteerMotor = 1;
        public static final int kFrontLeftSteerEncoder = 3;
        public static final SwerveModuleConstants kFrontLeft = new SwerveModuleConstants(
            kFrontLeftDriveMotor,
            kFrontLeftSteerMotor,
            kFrontLeftSteerEncoder,
            getFrontLeftOffset()
        );

        public static final int kFrontRightDriveMotor = 5;
        public static final int kFrontRightSteerMotor = 4;
        public static final int kFrontRightSteerEncoder = 6;
        public static final SwerveModuleConstants kFrontRight = new SwerveModuleConstants(
            kFrontRightDriveMotor,
            kFrontRightSteerMotor,
            kFrontRightSteerEncoder,
            getFrontRightOffset()
        );

        public static final int kBackLeftDriveMotor = 8;
        public static final int kBackLeftSteerMotor = 7;
        public static final int kBackLeftSteerEncoder = 9;
        public static final SwerveModuleConstants kBackLeft = new SwerveModuleConstants(
            kBackLeftDriveMotor,
            kBackLeftSteerMotor,
            kBackLeftSteerEncoder,
            getBackLeftOffset()
        );

        public static final int kBackRightDriveMotor = 11;
        public static final int kBackRightSteerMotor = 10;
        public static final int kBackRightSteerEncoder = 12;
        public static final SwerveModuleConstants kBackRight = new SwerveModuleConstants(
            kBackRightDriveMotor,
            kBackRightSteerMotor,
            kBackRightSteerEncoder,
            getBackRightOffset()
        );
    }
}
