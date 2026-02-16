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
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.SwerveModuleConstants;

// Constants found in properties files in deploy folder
public final class Constants {
    // Set to true to enable replaying log files
    public static final boolean kIsReplay = false;
    
    private static Properties m_properties;
    public static String kRobotName;

    public static void instantiateProperties() {
        m_properties = new Properties();
        kRobotName = kTeamNumberToName.get(RobotController.getTeamNumber());

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
    public static final HashMap<Integer, String> kTeamNumberToName;
    static {
        kTeamNumberToName = new HashMap<>();
        kTeamNumberToName.put(4784, "kitbot");
        kTeamNumberToName.put(4785, "hades");
        kTeamNumberToName.put(4786, "tusk");
    }

    public final class DeviceIds {
        public static int getFrontLeftSteerId() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.front-left-steer"));
        }

        public static int getFrontLeftDriveId() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.front-left-drive"));
        }

        public static int getFrontLeftSteerEncoderId() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.front-left-steer-encoder"));
        }

        public static int getFrontRightSteerId() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.front-right-steer"));
        }

        public static int getFrontRightDriveId() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.front-right-drive"));
        }

        public static int getFrontRightSteerEncoderId() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.front-right-steer-encoder"));
        }

        public static int getBackLeftSteerId() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.back-left-steer"));
        }

        public static int getBackLeftDriveId() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.back-left-drive"));
        }

        public static int getBackLeftSteerEncoderId() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.back-left-steer-encoder"));
        }

        public static int getBackRightSteerId() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.back-right-steer"));
        }

        public static int getBackRightDriveId() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.back-right-drive"));
        }

        public static int getBackRightSteerEncoderId() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.back-right-steer-encoder"));
        }

        public static int getPigeonId() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.pigeon"));
        }

        public static int getLeftLauncherID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.left-launcher"));
        }

        public static int getRightLauncherID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.right-launcher"));
        }

        public static int getKitbotIntakeID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.kitbot-intake"));
        }

        public static int getKitbotLauncherID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.kitbot-launcher"));
        }

        public static int getIndexerID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.indexer"));
        }

        public static int getIntakeDriverID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.intake-driver"));
        }

        public static int getIntakePivotID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.intake-pivot"));
        }

        public static int getClimbID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.climb"));
        }

        public static int getIntakeBeamBreakID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.intake-beam-break"));
        }

        public static int getIndexerBeamBreakID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.indexer-beam-break"));
        }

        // public static int getHoodID() {
        //     return Integer.parseInt(m_properties.getProperty("deviceIds.hood"));
        //}

        public static final int kLeftLauncherServo = 0;
        public static final int kRightLauncherServo = 1;
    }

    public final class OperatorConstants {
        public static int getDriverControllerPort() {
            return Integer.parseInt(m_properties.getProperty("operator.driver-controller-port"));
        }

        public static int getOperatorControllerPort() {
            return Integer.parseInt(m_properties.getProperty("operator.operator-controller-port"));
        }

        public static int getOperatorControllerDeadband() {
            return Integer.parseInt(m_properties.getProperty("operator.operator-controller-deadband"));
        }
        
        public static final int kThrottleAxis = Axis.kLeftY.value;
        public static final int kStrafeAxis = Axis.kLeftX.value;
        public static final int kSteerAxis = Axis.kRightX.value;

        public static double getDefaultSpeed() {
            return Double.parseDouble(m_properties.getProperty("operator.default-speed"));
        }

        public static double getSlowSpeed() {
            return Double.parseDouble(m_properties.getProperty("operator.slow-speed"));
        }
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

        public static final SwerveModuleConstants kFrontLeft = new SwerveModuleConstants(
            DeviceIds.getFrontLeftDriveId(),
            DeviceIds.getFrontLeftSteerId(),
            DeviceIds.getFrontLeftSteerEncoderId(),
            getFrontLeftOffset()
        );

        public static final SwerveModuleConstants kFrontRight = new SwerveModuleConstants(
            DeviceIds.getFrontRightDriveId(),
            DeviceIds.getFrontRightSteerId(),
            DeviceIds.getFrontRightSteerEncoderId(),
            getFrontRightOffset()
        );

        public static final SwerveModuleConstants kBackLeft = new SwerveModuleConstants(
            DeviceIds.getBackLeftDriveId(),
            DeviceIds.getBackLeftSteerId(),
            DeviceIds.getBackLeftSteerEncoderId(),
            getBackLeftOffset()
        );

        public static final SwerveModuleConstants kBackRight = new SwerveModuleConstants(
            DeviceIds.getBackRightDriveId(),
            DeviceIds.getBackRightSteerId(),
            DeviceIds.getBackRightSteerEncoderId(),
            getBackRightOffset()
        );
    }

    public final class LauncherConstants {
        public static double getGearRatio() {
            return Double.parseDouble(m_properties.getProperty("launcher.gear-ratio"));
        }

        public static double getHoodSetpointTolerance() {
            return Double.parseDouble(m_properties.getProperty("launcher.hood-setpoint-tolerance"));
        }

        public static double getHoodManualModifier() {
            return Double.parseDouble(m_properties.getProperty("launcher.hood-manual-modifier"));
        }

        public static double getHoodMinPosition() {
            return Double.parseDouble(m_properties.getProperty("launcher.hood-min-position"));
        }

        public static double getHoodMaxPosition() {
            return Double.parseDouble(m_properties.getProperty("launcher.hood-max-position"));
        }
        
        public static double getKP() {
            return Double.parseDouble(m_properties.getProperty("launcher.kp"));
        }

        public static double getKI() {
            return Double.parseDouble(m_properties.getProperty("launcher.ki"));
        }

        public static double getKD() {
            return Double.parseDouble(m_properties.getProperty("launcher.kd"));
        }

        public static double getOffVoltage() {
            return Double.parseDouble(m_properties.getProperty("launcher.off-voltage"));
        }

        public static double getLaunchVoltage() {
            return Double.parseDouble(m_properties.getProperty("launcher.launch-voltage"));
        }
    }

    public final class IndexerConstants {
        public static double getGearRatio() {
            return Double.parseDouble(m_properties.getProperty("indexer.gear-ratio"));
        }

        public static double getKP() {
            return Double.parseDouble(m_properties.getProperty("indexer.kp"));
        }

        public static double getKI() {
            return Double.parseDouble(m_properties.getProperty("indexer.ki"));
        }

        public static double getKD() {
            return Double.parseDouble(m_properties.getProperty("indexer.kd"));
        }

        public static double getOffVoltage() {
            return Double.parseDouble(m_properties.getProperty("indexer.off-voltage"));
        }

        public static double getIndexVoltage() {
            return Double.parseDouble(m_properties.getProperty("indexer.index-voltage"));
        }
    }

    public final class IntakeConstants {
        public static double getDriverGearRatio() {
            return Double.parseDouble(m_properties.getProperty("intake.driver-gear-ratio"));
        }
        public static double getPivotGearRatio() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot-gear-ratio"));
        }
        
        public static double getDriverKP() {
            return Double.parseDouble(m_properties.getProperty("intake.driver-kp"));
        }

        public static double getDriverKI() {
            return Double.parseDouble(m_properties.getProperty("intake.driver-ki"));
        }

        public static double getDriverKD() {
            return Double.parseDouble(m_properties.getProperty("intake.driver-kd"));
        }

        public static double getPivotKP() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot-kp"));
        }

        public static double getPivotKI() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot-ki"));
        }

        public static double getPivotKD() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot-kd"));
        }
        
        public static double getDriverOffVoltage() {
            return Double.parseDouble(m_properties.getProperty("intake.driver-off-voltage"));
        }

        public static double getDriverIntakeVoltage() {
            return Double.parseDouble(m_properties.getProperty("intake.driver-intake-voltage"));
        }

        public static double getPivotSetpointTolerance() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot-setpoint-tolerance"));
        }

        public static double getPivotManualMultiplier() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot-manual-multiplier"));
        }

        public static double getPivotMinPosition() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot-min-position"));
        }

        public static double getPivotMaxPosition() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot-max-position"));
        }

        public static double getPivotHomePosition() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot-home-position"));
        }

        public static double getPivotOutPosition() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot-out-position"));
        }
    }

    public final class ClimbConstants {
        public static double getGearRatio() {
            return Double.parseDouble(m_properties.getProperty("climb.gear-ratio"));
        }

        public static double getSprocketCircumference() {
            return Math.PI * Double.parseDouble(m_properties.getProperty("climb.sprocket-circumference"));
        }
        
        public static double getSetpointTolerance() {
            return Double.parseDouble(m_properties.getProperty("climb.setpoint-tolerance"));
        }

        public static double getManualMultiplier() {
            return Double.parseDouble(m_properties.getProperty("climb.manual-multiplier"));
        }

         public static double getKP() {
            return Double.parseDouble(m_properties.getProperty("climb.kp"));
        }

        public static double getKI() {
            return Double.parseDouble(m_properties.getProperty("climb.ki"));
        }

        public static double getKD() {
            return Double.parseDouble(m_properties.getProperty("climb.kd"));
        }

        public static double getMinPosition() {
            return Double.parseDouble(m_properties.getProperty("climb.min-position"));
        }

        public static double getMaxPosition() {
            return Double.parseDouble(m_properties.getProperty("climb.max-position"));
        }

        public static double getHomePosition() {
            return Double.parseDouble(m_properties.getProperty("climb.home-position"));
        }

        public static double getL1Position() {
            return Double.parseDouble(m_properties.getProperty("climb.L1-position"));
        }
    }
}
