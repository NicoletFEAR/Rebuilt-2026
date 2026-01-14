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

public final class Constants {
    private static Properties m_properties;

    public static void instantiateProperties() {
        m_properties = new Properties();

        // TODO: Add detection for which property file to use based on MAC address
        try (InputStream input = new FileInputStream(new File(Filesystem.getDeployDirectory(), "competition.properties"))) {
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
            return Units.feetToMeters(Double.parseDouble(m_properties.getProperty("drive.max-rotations-per-second")));
        }

        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = Units.inchesToMeters(20.753888);

        // Distance between centers of front and back wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(20.753888);

        // Defined as half the diagonal of the drivebase
        public static final double kDrivebaseRadius = Math.hypot(kTrackWidth, kWheelBase) / 2;

        // Positions of all the swerve modules relative to the center of the drivebase
        public static final Translation2d[] kModuleTranslations = {
            new Translation2d(kWheelBase  / 2, kTrackWidth  / 2),
            new Translation2d(kWheelBase  / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth  / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        };

        public static final SwerveModuleState[] kXWheels = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-135))
        };

        // sds L2 gear ratio
        // TODO: Remember to change if we modify the swerve modules
        public static final double kDriveGearRatio = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)); 

        public static final double kTurnGearRatio = 150.0 / 7.0; // MK4i turning ratio

        // Run the Wheel Characterization command to find the wheel diameter
        public static final double kWheelDiameter = 0.09694774659544067; 

        public static final double kDriveRevToMeters = kDriveGearRatio / (Math.PI * kWheelDiameter);

        public static final double kTurnRotationsToDegrees = 360.0 / kTurnGearRatio;

        public static final RobotConfig kRobotConfig = new RobotConfig(Units.lbsToKilograms(115),
            // TODO: Calculate MOI based on the values given by Elijah from the CAD
            6.883,
            new ModuleConfig(DriveConstants.kWheelDiameter / 2,
                DriveConstants.kMaxModuleSpeed,
                1,
                DCMotor.getKrakenX60(1).withReduction(kDriveGearRatio),
                103,
                1),
            DriveConstants.kModuleTranslations
        );

        // TODO: Tune PID
        public static double drivekp = 2.7141;
        public static double driveki = 0.0;
        public static double drivekd = 0.0;
        public static double driveks = 0.067703;
        public static double drivekv = 2.4746;
        public static double driveka = 0.36888;
        public static double driverampRate = 0.1;

        public static double turnkp = 0.02;
        public static double turnki = 0.0;
        public static double turnkd = 0.01;
        public static double turnkff = 0.0;

        public static final int kPigeonId = 13;

        public static final int kFrontLeftDriveMotor = 2;
        public static final int kFrontLeftSteerMotor = 1;
        public static final int kFrontLeftSteerEncoder = 3;
        // TODO: Calibrate wheels
        public static final double kFrontLeftOffset = -0.730712890625;
        public static final SwerveModuleConstants kFrontLeft = new SwerveModuleConstants(
            kFrontLeftDriveMotor,
            kFrontLeftSteerMotor,
            kFrontLeftSteerEncoder,
            kFrontLeftOffset
        );

        public static final int kFrontRightDriveMotor = 5;
        public static final int kFrontRightSteerMotor = 4;
        public static final int kFrontRightSteerEncoder = 6;
        public static final double kFrontRightSteerOffset = -0.986328125;
        public static final SwerveModuleConstants kFrontRight = new SwerveModuleConstants(
            kFrontRightDriveMotor,
            kFrontRightSteerMotor,
            kFrontRightSteerEncoder,
            kFrontRightSteerOffset
        );

        public static final int kBackLeftDriveMotor = 8;
        public static final int kBackLeftSteerMotor = 7;
        public static final int kBackLeftSteerEncoder = 9;
        public static final double kBackLeftSteerOffset = -0.215576171875;
        public static final SwerveModuleConstants kBackLeft = new SwerveModuleConstants(
            kBackLeftDriveMotor,
            kBackLeftSteerMotor,
            kBackLeftSteerEncoder,
            kBackLeftSteerOffset
        );

        public static final int kBackRightDriveMotor = 11;
        public static final int kBackRightSteerMotor = 10;
        public static final int kBackRightSteerEncoder = 12;
        public static final double kBackRightSteerOffset = -0.705810546875;
        public static final SwerveModuleConstants kBackRight = new SwerveModuleConstants(
            kBackRightDriveMotor,
            kBackRightSteerMotor,
            kBackRightSteerEncoder,
            kBackRightSteerOffset
        );
    }
}
