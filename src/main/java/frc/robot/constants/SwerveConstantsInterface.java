package frc.robot.constants;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.SwerveModuleConstants;

public abstract interface SwerveConstantsInterface {
    int getIsBatteryInBack();
    boolean getHasCanivore();
    boolean getUsesDriveKrakens();
    double getSwerveDeadband();

    double getMaxModuleSpeed();
    double getMaxRotationsPerSecond();

    // Distance between centers of right and left wheels on robot
    double getTrackWidth();
    // distance between centers of front and back wheels on robot
    double getWheelBase();
    // Defined as half the diagonal of the drivebase
    double getDrivebaseRadius();
    double getDriveGearRatio();
    double getTurnGearRatio();
    // TODO: Run the WheelCharacterization command to find the wheel diameter
    double getWheelDiameter();
    double getDriveRevToMeters();
    double getTurnRotationsToDegrees();
    double getWheelCof();
    double getCurrentLimit();

    // TODO: Tune PID
    double getDriveKP();
    double getDriveKI();
    double getDriveKD();
    double getDriveKS();
    double getDriveKV();
    double getDriveKA();

    double getRampRate();

    double getTurnKP();
    double getTurnKI();
    double getTurnKD();
    double getTurnKV();
    double getTurnKS();
    double getTurnKA();

    double getFrontLeftOffset();
    double getFrontRightOffset();
    double getBackLeftOffset();
    double getBackRightOffset();

    double getAutoTargetKP();
    double getAutoTargetKI();
    double getAutoTargetKD();

    Translation2d[] getModuleTranslations();

    RobotConfig getRobotConfig();

    SwerveModuleConstants getFrontLeft();
    SwerveModuleConstants getFrontRight();
    SwerveModuleConstants getBackLeft();
    SwerveModuleConstants getBackRight();
}
