// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.controllers.Axis;
import frc.robot.controllers.ControllerType;
import frc.robot.util.SwerveModuleConstants;

// Constants found in properties files in deploy folder
public final class Constants {
    // Set to true to enable replaying log files
    public static final boolean kIsReplay = false;

    private static Properties m_properties;
    public static String kRobotName;

    public static void instantiateProperties() {
        m_properties = new Properties();

        if (Robot.isReal()) {
            kRobotName = kTeamNumberToName.get(RobotController.getTeamNumber());
        } else {
            kRobotName = "tusk";
        }

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

        public static int getIntakeBeamBreakID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.intake-beam-break"));
        }

        public static int getIndexerBeamBreakID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.indexer-beam-break"));
        }

         public static int getHoodID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.hood"));
        }

        public static int getHoodEncoderID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.hood-encoder"));
        }

        public static int getLedID() {
            return Integer.parseInt(m_properties.getProperty("deviceIds.led"));
        }
    }

    public final class OperatorConstants {
        public static int getDriverControllerPort() {
            return Integer.parseInt(m_properties.getProperty("operator.driver-controller-port"));
        }

        public static int getOperatorControllerPort() {
            return Integer.parseInt(m_properties.getProperty("operator.operator-controller-port"));
        }

        public static double getOperatorControllerDeadband() {
            return Double.parseDouble(m_properties.getProperty("operator.operator-controller-deadband"));
        }

        public static final Axis operatorAxis = new Axis(DriveConstants.getControllerType());
        public static final int kThrottleAxis = operatorAxis.getAxis("kLeftY");
        public static final int kStrafeAxis = operatorAxis.getAxis("kLeftX");
        public static final int kSteerAxis = operatorAxis.getAxis("kRightX");

        public static ControllerType getControllerType() {
            return ControllerType.valueOf(m_properties.getProperty("operator.controller-type"));
        }

        public static double getDefaultSpeed() {
            return Double.parseDouble(m_properties.getProperty("operator.default-speed"));
        }

        public static double getSlowSpeed() {
            return Double.parseDouble(m_properties.getProperty("operator.slow-speed"));
        }
    }

    public final class DriveConstants {

        public static int isBatteryInBack() {
            if (m_properties.getProperty("battery-back").equals("true"))
                return -1;
            else
                return 1;
        }

        public static ControllerType getControllerType() {
            return ControllerType.valueOf(m_properties.getProperty("drive.controller-type"));
        }

        public static boolean usesDriveKrakens() {
            return m_properties.getProperty("drive.uses-drive-krakens").equals("true");
        }

        public static double getSwerveDeadband() {
            return 0.075;
        }

        public static double getMaxModuleSpeed() {
            return Double.parseDouble(m_properties.getProperty("drive.max-module-speed"));
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
            return Math.hypot(getTrackWidth(), getWheelBase()) / 2.0;
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

        public static double getRotationTolerance() {
            return Double.parseDouble(m_properties.getProperty("drive.rotation-tolerance"));
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

        public static double getAutoTargetKP() {
            return Double.parseDouble(m_properties.getProperty("drive.auto-target.kp"));
        }

        public static double getAutoTargetKI() {
            return Double.parseDouble(m_properties.getProperty("drive.auto-target.ki"));
        }

        public static double getAutoTargetKD() {
            return Double.parseDouble(m_properties.getProperty("drive.auto-target.kd"));
        }

        public static double getPathTranslationKP() {
            return Double.parseDouble(m_properties.getProperty("drive.path-translation-kp"));
        }

        public static double getPathTranslationKI() {
            return Double.parseDouble(m_properties.getProperty("drive.path-translation-ki"));
        }

        public static double getPathTranslationKD() {
            return Double.parseDouble(m_properties.getProperty("drive.path-translation-kd"));
        }

        public static double getPathRotationKP() {
            return Double.parseDouble(m_properties.getProperty("drive.path-rotation-kp"));
        }

        public static double getPathRotationKI() {
            return Double.parseDouble(m_properties.getProperty("drive.path-rotation-ki"));
        }

        public static double getPathRotationKD() {
            return Double.parseDouble(m_properties.getProperty("drive.path-rotation-kd"));
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
                DriveConstants.usesDriveKrakens() ?
                    DCMotor.getKrakenX60(1).withReduction(getDriveGearRatio()) :
                    DCMotor.getNEO(1).withReduction(getDriveGearRatio()),
                getCurrentLimit(),
                1),
            DriveConstants.kModuleTranslations
        );

        public static final SwerveModuleConstants kFrontLeft = new SwerveModuleConstants(
            DeviceIds.getFrontLeftDriveId(),
            DeviceIds.getFrontLeftSteerId(),
            DeviceIds.getFrontLeftSteerEncoderId(),
            getFrontLeftOffset(),
            true
        );

        public static final SwerveModuleConstants kFrontRight = new SwerveModuleConstants(
            DeviceIds.getFrontRightDriveId(),
            DeviceIds.getFrontRightSteerId(),
            DeviceIds.getFrontRightSteerEncoderId(),
            getFrontRightOffset(),
            true
        );

        public static final SwerveModuleConstants kBackLeft = new SwerveModuleConstants(
            DeviceIds.getBackLeftDriveId(),
            DeviceIds.getBackLeftSteerId(),
            DeviceIds.getBackLeftSteerEncoderId(),
            getBackLeftOffset(),
            true
        );

        public static final SwerveModuleConstants kBackRight = new SwerveModuleConstants(
            DeviceIds.getBackRightDriveId(),
            DeviceIds.getBackRightSteerId(),
            DeviceIds.getBackRightSteerEncoderId(),
            getBackRightOffset(),
            true
        );

        public static final double kVerticalMidfieldPosition = 4.0d;
        public static final double kInAllianceZoneBlue = Meters.convertFrom(182.11, Inches);
        public static final double kInAllianceZoneRed = Meters.convertFrom(469.11, Inches);
        public static final Translation2d kBlueHubPosition = new Translation2d(4.619, 4.033);
        public static final Translation2d kRedHubPosition = new Translation2d(11.936, 4.033);
        public static final Translation2d kBlueLeftPassingPosition = new Translation2d(Meters.convertFrom(43, Inches), Meters.convertFrom(82.91, Inches));
        public static final Translation2d kBlueRightPassingPosition = new Translation2d(Meters.convertFrom(43, Inches), Meters.convertFrom(234.77, Inches));
        public static final Translation2d kRedLeftPassingPosition = new Translation2d(Meters.convertFrom(608.22, Inches), Meters.convertFrom(234.77, Inches));
        public static final Translation2d kRedRightPassingPosition = new Translation2d(Meters.convertFrom(608.22, Inches), Meters.convertFrom(82.91, Inches));
    }

    public final class LauncherConstants {
        // Formatted as pairs where value 1 is the distance from the hub and value 2 is the desired hood position
        // Must be sorted from low to high distance from the hub
        public static InterpolatingDoubleTreeMap kAutoAimHoodPositions = new InterpolatingDoubleTreeMap();
        static {
            kAutoAimHoodPositions.put(0.0d, 0.0d);
            kAutoAimHoodPositions.put(1.616208d, 0.024414d);
            kAutoAimHoodPositions.put(2.164127d, 0.174316d);
            kAutoAimHoodPositions.put(2.90789d, 0.419922d);
            kAutoAimHoodPositions.put(3.045959d, 0.420654d);
            kAutoAimHoodPositions.put(3.225971d, 0.612549d);
            kAutoAimHoodPositions.put(3.564832d, 0.555908d);
            kAutoAimHoodPositions.put(4.008246d, 0.613037d);
            kAutoAimHoodPositions.put(4.648431d, 0.687256d);
            kAutoAimHoodPositions.put(5.204873d, 0.836426d);
        }

        // Formatted as pairs where value 1 is the distance from the hub and value 2 is the desired launcher speed modifier
        public static InterpolatingDoubleTreeMap kAutoAimSpeeds = new InterpolatingDoubleTreeMap();
        static {
            kAutoAimSpeeds.put(0.0d, 0.7d);
            kAutoAimSpeeds.put(1.616208d, 0.8d);
            kAutoAimSpeeds.put(2.164127d, 0.837796d);
            kAutoAimSpeeds.put(2.90789d, 0.8891098d);
            kAutoAimSpeeds.put(3.045959d, 0.8891098d);
            kAutoAimSpeeds.put(3.225971d, 0.837796d);
            kAutoAimSpeeds.put(3.564832d, 0.930613d);
            kAutoAimSpeeds.put(4.008246d, 1.0d);
            kAutoAimSpeeds.put(4.648431d, 1.0d);
            kAutoAimSpeeds.put(5.204873d, 1.0d);
        }

        public static InterpolatingDoubleTreeMap kAutoAimTof = new InterpolatingDoubleTreeMap();
        static {
            kAutoAimTof.put(1.5, 1.3);
            kAutoAimTof.put(2.0, 1.58);
            kAutoAimTof.put(2.5, 1.48);
            kAutoAimTof.put(3.0, 1.45);
        }

        public static double getGearRatio() {
            return Double.parseDouble(m_properties.getProperty("launcher.gear-ratio"));
        }

        public static double getHoodGearRatio() {
            return Double.parseDouble(m_properties.getProperty("launcher.hood.gear-ratio"));
        }

        public static double getHoodSetpointTolerance() {
            return Double.parseDouble(m_properties.getProperty("launcher.hood.setpoint-tolerance"));
        }

        public static double getHoodManualModifier() {
            return Double.parseDouble(m_properties.getProperty("launcher.hood.manual-modifier"));
        }

        public static double getHoodHomePosition() {
            return Double.parseDouble(m_properties.getProperty("launcher.hood.home-position"));
        }

        public static double getHoodMinPosition() {
            return Double.parseDouble(m_properties.getProperty("launcher.hood.min-position"));
        }

        public static double getHoodMaxPosition() {
            return Double.parseDouble(m_properties.getProperty("launcher.hood.max-position"));
        }

        public static double getHoodKP() {
            return Double.parseDouble(m_properties.getProperty("launcher.hood.kp"));
        }

        public static double getHoodKI() {
            return Double.parseDouble(m_properties.getProperty("launcher.hood.ki"));
        }

        public static double getHoodKD() {
            return Double.parseDouble(m_properties.getProperty("launcher.hood.kd"));
        }

        public static double getHoodOffset() {
            return Double.parseDouble(m_properties.getProperty("launcher.hood.offset"));
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

        public static double getVelocityTolerance() {
            return Double.parseDouble(m_properties.getProperty("launcher.velocity-tolerance"));
        }

        public static double getIdleVelocity() {
            return Double.parseDouble(m_properties.getProperty("launcher.idle-velocity"));
        }

        public static double getLaunchVelocity() {
            return Double.parseDouble(m_properties.getProperty("launcher.launch-velocity"));
        }

        public static double getReverseVelocity() {
            return Double.parseDouble(m_properties.getProperty("launcher.reverse-velocity"));
        }

        public static double getOffVelocity() {
            return Double.parseDouble(m_properties.getProperty("launcher.off-velocity"));
        }

        public static final double kHoodPassingPosition = 0.84d;
        public static final double kPassSpeedModifier = 1.0d;
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

        public static double getExtakeVoltage() {
            return Double.parseDouble(m_properties.getProperty("indexer.extake-voltage"));
        }
    }

    public final class IntakeConstants {
        public static double getDriverGearRatio() {
            return Double.parseDouble(m_properties.getProperty("intake.driver.gear-ratio"));
        }
        public static double getPivotGearRatio() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot.gear-ratio"));
        }

        public static double getDriverKP() {
            return Double.parseDouble(m_properties.getProperty("intake.driver.kp"));
        }

        public static double getDriverKI() {
            return Double.parseDouble(m_properties.getProperty("intake.driver.ki"));
        }

        public static double getDriverKD() {
            return Double.parseDouble(m_properties.getProperty("intake.driver.kd"));
        }

        public static double getPivotKP() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot.kp"));
        }

        public static double getPivotKI() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot.ki"));
        }

        public static double getPivotKD() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot.kd"));
        }

        public static double getDriverOffVoltage() {
            return Double.parseDouble(m_properties.getProperty("intake.driver.off-voltage"));
        }

        public static double getDriverIntakeVoltage() {
            return Double.parseDouble(m_properties.getProperty("intake.driver.intake-voltage"));
        }

        public static double getDriverExtakeVoltage() {
            return Double.parseDouble(m_properties.getProperty("intake.driver.extake-voltage"));
        }

        public static double getIntakeStuckOnBallThreshold() {
            return Double.parseDouble(m_properties.getProperty("intake.stuck-on-ball-threshold"));
        }

        public static double getPivotSetpointTolerance() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot.setpoint-tolerance"));
        }

        public static double getPivotManualModifier() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot.manual-modifier"));
        }

        public static double getPivotMinPosition() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot.min-position"));
        }

        public static double getPivotMaxPosition() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot.max-position"));
        }

        public static double getPivotHomePosition() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot.home-position"));
        }

        public static double getPivotJostlePosition() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot.jostle-position"));
        }

        public static double getPivotHoldPosition() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot.hold-position"));
        }

        public static double getPivotOutPosition() {
            return Double.parseDouble(m_properties.getProperty("intake.pivot.out-position"));
        }
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
