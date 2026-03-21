package frc.robot.subsystems.turn;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.DeviceIds;
import frc.robot.constants.DriveConstants;
import frc.robot.util.CANId;

public enum TurnName {
    FRONT_LEFT("Front Left"),
    FRONT_RIGHT("Front Right"),
    REAR_LEFT("Rear Left"),
    REAR_RIGHT("Rear Right"),
    ;

    private final String m_name;

    TurnName(String name) {
        m_name = name;
    }

    @Override
    public String toString() {
        return m_name;
    }

    public CANId getAbsoluteEncoderId() {
        return switch (this) {
            case FRONT_LEFT -> DeviceIds.kFrontLeftSteerAbsoluteEncoder;
            case FRONT_RIGHT -> DeviceIds.kFrontRightSteerAbsoluteEncoder;
            case REAR_LEFT -> DeviceIds.kRearLeftSteerAbsoluteEncoder;
            case REAR_RIGHT -> DeviceIds.kRearRightSteerAbsoluteEncoder;
        };
    }

    public Angle getAbsoluteEncoderOffset(DriveConstants driveConstants) {
        return switch (this) {
            case FRONT_LEFT -> driveConstants.kFrontLeftOffset;
            case FRONT_RIGHT -> driveConstants.kFrontRightOffset;
            case REAR_LEFT -> driveConstants.kRearLeftOffset;
            case REAR_RIGHT -> driveConstants.kRearRightOffset;
        };
    }

    public CANId getMotorId() {
        return switch (this) {
            case FRONT_LEFT -> DeviceIds.kFrontLeftSteer;
            case FRONT_RIGHT -> DeviceIds.kFrontRightSteer;
            case REAR_LEFT -> DeviceIds.kRearLeftSteer;
            case REAR_RIGHT -> DeviceIds.kRearRightSteer;
        };
    }
}
