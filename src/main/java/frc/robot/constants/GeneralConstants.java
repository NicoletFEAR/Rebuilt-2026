package frc.robot.constants;

public class GeneralConstants {
    // Set to true to enable replaying log files
    public static final boolean kIsReplay = false;

    // How frequently the state of the robot updates during simulations
    // Set to 0.02 to represent once every loop (20 ms)
    public static final double kdt = 0.02;

    public class DrivebaseMotorIds {
        public static final int frontLeftSteerMotorId = 1;
        public static final int frontLeftDriveMotorId = 2;
        public static final int frontLeftSteerEncoderId = 3;

        public static final int frontRightSteerMotorId = 4;
        public static final int frontRightDriveMotorId = 5;
        public static final int frontRightSteerEncoderId = 6;

        public static final int backLeftSteerMotorId = 10;
        public static final int backLeftDriveMotorId = 11;
        public static final int backLeftSteerEncoderId = 12;

        public static final int backRightSteerMotorId = 7;
        public static final int backRightDriveMotorId = 8;
        public static final int backRightSteerEncoderId = 9;

        public static final int pigeonId = 13;
    }

    public class GeneralOperatorConstants {
        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1;
        public static final double operatorControllerDeadband = 0.1d;
        public static final double defaultSpeed = 1.0d;
        public static final double slowSpeed = 0.4d;
    }

}
