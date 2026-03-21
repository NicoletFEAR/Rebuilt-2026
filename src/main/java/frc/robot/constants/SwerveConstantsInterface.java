package frc.robot.constants;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.SwerveModuleConstants;

/**
 * This interface is used to set all of the constants needed for
 * a swerve drive. All swerve drive constants must extend this interface
 */
public interface SwerveConstantsInterface {
    /**
     * Gets a numerical value for whether the battery is in the back
     * @return -1 or 1 depending on if the battery is in the back
     */
    public int getIsBatteryInBack();

    /**
     * Gets whether the robot has a CANivore
     * @return true is the robot has a CANivore and false if otherwise
     */
    public boolean getHasCanivore();

    /**
     * Gets whether the drivebase uses Krakens for the drive motors
     * @return true is Krakens are used and false if not
     */
    public boolean getUsesDriveKrakens();

    /**
     * Gets the swerve deadband of the robot
     * @return the swerve deadband as a double
     */
    public double getSwerveDeadband();

    /**
     * Gets the max modules speeds of the drivebase
     * <p>
     * Prop the drivebase up so the wheels aren't touching the ground
     * and run the drive motors at 12 volts and measure the max speed
     * of the modules in m/s
     * @return the max module speed in m/s as a double
     */
    public double getMaxModuleSpeed();

    /**
     * Gets the max rotations of the module wheels per second
     * @return the max rotations of the wheel per second as a double
     */
    public double getMaxRotationsPerSecond();

    /**
     * Gets the distance between the centers of the right and
     * left wheels on the robot
     * @return gets the track width in inches as a double
     */
    public double getTrackWidth();

    /**
     * Gets the distance between the front and back wheels
     * @return gets the wheel base in inches as a double
     */
    public double getWheelBase();

    /**
     * Gets the half of the diagonal distance of the drivebase
     * @return gets the hypotenuse of the Track Width and Wheel Base
     */
    public double getDrivebaseRadius();

    /**
     * Gets the gear ratio of the drive motor
     * @return the gear ratio as a double(can use a fraction for easier calculation)
     */
    public double getDriveGearRatio();

    /**
     * Gets the gear ratio of the turning/steering motor
     * @return the gear ratio as a double(can use a fraction for easier calculation)
     */
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
