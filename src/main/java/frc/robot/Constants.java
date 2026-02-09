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

// Constants found in properties files in deploy folder
public final class Constants {
    private static Properties m_properties;
    public static String kRobotName;

    public static void instantiateProperties() {
        m_properties = new Properties();
        kRobotName = kMacToName.get(Utils.getMacAddress());
        System.out.println(Utils.getMacAddress());

        try (InputStream input = new FileInputStream(new File(Filesystem.getDeployDirectory(), kRobotName + ".properties"))) {
            m_properties.load(input);
        } catch (IOException ex) {
            DriverStation.reportError("Constants file not found", ex.getStackTrace());
        }
    }

    public static boolean hasCANivore() {
        return m_properties.getProperty("has-canivore").equals("true");
    }

    // How frequently the state of the robot updates during simulations
    // Set to 0.02 to represent once every loop (20 ms)
    public static final double kdt = 0.02;

    // These are the unique identifiers for each RoboRIO we plan to use
    public static final HashMap<String, String> kMacToName;
    static {
        kMacToName = new HashMap<>();
        kMacToName.put("00:80:2F:40:69:FC", "kitbot");
        kMacToName.put("00:80:2F:17:B5:2A", "hades");
        kMacToName.put("00:80:2F:40:6C:8A", "tusk");
        kMacToName.put("1C:5C:37:32:5C:14", "tusk");
    }

    public final class MotorIds {
        public static final int kLauncherLeft = 14;
        public static final int kLauncherRight = 15;

        public static final int kKitbotIntake = 14;
        public static final int kKitbotLauncher = 15;

        public static final int kIndexLauncher = 16;
        public static final int kIndexIntake = 17;

        public static final int kIntakeDriver = 18;
        public static final int kIntakePivot = 19;

        public static final int kClimb = 20;
    }

    public final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kOperatorControllerDeadband = 0.1;

        public static final int kThrottleAxis = Axis.kLeftY.value;
        public static final int kStrafeAxis = Axis.kLeftX.value;
        public static final int kSteerAxis = Axis.kRightX.value;

        public static final double kDefaultSpeed = 1;
        public static final double kSlowSpeed = 0.4;
    }

    public final class DriveConstants {
        public static boolean usesDriveKrakens() {
            return m_properties.getProperty("drive.uses-drive-krakens").equals("true");
        }

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

        public static double getTurnKS() {
            return Double.parseDouble(m_properties.getProperty("drive.turn-ks"));
        }

        public static double getTurnKV() {
            return Double.parseDouble(m_properties.getProperty("drive.turn-kv"));
        }

        public static double getTurnKA() {
            return Double.parseDouble(m_properties.getProperty("drive.turn-ka"));
        }
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

        public static final int kBackLeftDriveMotor = 11;
        public static final int kBackLeftSteerMotor = 10;
        public static final int kBackLeftSteerEncoder = 12;
        public static final SwerveModuleConstants kBackLeft = new SwerveModuleConstants(
            kBackLeftDriveMotor,
            kBackLeftSteerMotor,
            kBackLeftSteerEncoder,
            getBackLeftOffset()
        );

        public static final int kBackRightDriveMotor = 8;
        public static final int kBackRightSteerMotor = 7;
        public static final int kBackRightSteerEncoder = 9;
        public static final SwerveModuleConstants kBackRight = new SwerveModuleConstants(
            kBackRightDriveMotor,
            kBackRightSteerMotor,
            kBackRightSteerEncoder,
            getBackRightOffset()
        );
    }

    public final class LauncherConstants {
        public static final double kGearRatio = 1.0;

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public final class IndexerConstants {
         public static final double kGearRatio = 1.0;

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public final class IntakeConstants {
        public static final double kDriverGearRatio = 1.0;
        public static final double kPivotGearRatio = 1.0;
        
        public static final double kDriverKP = 1.0;
        public static final double kDriverKI = 0.0;
        public static final double kDriverKD = 0.0;
        public static final double kPivotKP = 1.0;
        public static final double kPivotKI = 0.0;
        public static final double kPivotKD = 0.0;

        public static final double kPivotSetpointTolerance = 0.025;
        public static final double kPivotManualMultiplier = 0.005;

        public static final double kPivotMinPosition = 0.0;
        public static final double kPivotMaxPosition = 1000.0;
        public static final double kPivotHomePosition = 0.0;
    }

    public final class ClimbConstants {
        public static final double kGearRatio = 25.0;
        public static final double kSprocketCircumference = 1.79 * Math.PI;
        public static final double kSetpointTolerance = 0.05;
        public static final double kManualMultiplier = 0.005;

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kMinPosition = 0.0;
        public static final double kMaxPosition = 1000.0;

        public static final double kHomePosition = 0.0;
        public static final double kL1Position = 15.0;
    }
}
