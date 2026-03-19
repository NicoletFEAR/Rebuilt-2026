package frc.robot.constants;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.SwerveModuleConstants;

public interface SwerveConstantsInterface {

    public int getIsBatteryInBack();
    public boolean getHasCanivore();
    public boolean getUsesDriveKrakens();
    public double getSwerveDeadband();

    public double getMaxModuleSpeed();
    public double getMaxRotationsPerSecond();

    // Distance between centers of right and left wheels on robot
    public double getTrackWidth();
    // distance between centers of front and back wheels on robot
    public double getWheelBase();
    // Defined as half the diagonal of the drivebase
    public double getDrivebaseRadius();
    public double getDriveGearRatio();
    public double getTurnGearRatio();
    // TODO: Run the WheelCharacterization command to find the wheel diameter
    public double getWheelDiameter();
    public double getDriveRevToMeters();
    public double getTurnRotationsToDegrees();
    public double getWheelCof();
    public double getCurrentLimit();

    // TODO: Tune PID
    public double getDriveKP();
    public double getDriveKI();
    public double getDriveKD();
    public double getDriveKS();
    public double getDriveKV();
    public double getDriveKA();

    public double getRampRate();

    public double getTurnKP();
    public double getTurnKI();
    public double getTurnKD();
    public double getTurnKV();
    public double getTurnKS();
    public double getTurnKA();

    public double getFrontLeftOffset();
    public double getFrontRightOffset();
    public double getBackLeftOffset();
    public double getBackRightOffset();

    public double getAutoTargetKP();
    public double getAutoTargetKI();
    public double getAutoTargetKD();

    public Translation2d[] getModuleTranslations();

    public RobotConfig getRobotConfig();

    public SwerveModuleConstants getFrontLeft();
    public SwerveModuleConstants getFrontRight();
    public SwerveModuleConstants getBackLeft();
    public SwerveModuleConstants getBackRight();
}
