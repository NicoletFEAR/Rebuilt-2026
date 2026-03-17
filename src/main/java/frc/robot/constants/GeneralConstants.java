package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public class GeneralConstants {
    // Set to true to enable replaying log files
    public static final boolean kIsReplay = false;

    // How frequently the state of the robot updates during simulations
    // Set to 0.02 to represent once every loop (20 ms)
    public static final double kdt = 0.02;

    public static final Translation2d kHubPosition = new Translation2d(4.619, 4.033);
    
    public class DrivebaseMotorIds {
        public static final int kFrontLeftSteerMotorId = 1;
        public static final int kFrontLeftDriveMotorId = 2;
        public static final int kFrontLeftSteerEncoderId = 3;

        public static final int kFrontRightSteerMotorId = 4;
        public static final int kFrontRightDriveMotorId = 5;
        public static final int kFrontRightSteerEncoderId = 6;

        public static final int kBackLeftSteerMotorId = 10;
        public static final int kBackLeftDriveMotorId = 11;
        public static final int kBackLeftSteerEncoderId = 12;

        public static final int kBackRightSteerMotorId = 7;
        public static final int kBackRightDriveMotorId = 8;
        public static final int kBackRightSteerEncoderId = 9;

        public static final int kPigeonId = 13;
    }

    public class GeneralOperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kOperatorControllerDeadband = 0.1d;
        public static final double kDefaultSpeed = 1.0d;
        public static final double kSlowSpeed = 0.4d;
    }

}
