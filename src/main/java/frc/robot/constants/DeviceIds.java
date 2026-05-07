package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

import frc.robot.util.CANId;

public final class DeviceIds {
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;

    public static final CANBus RIO_BUS = new CANBus("rio");
    public static final CANBus CANIVORE_BUS = new CANBus("*");

    public static final CANId FRONT_LEFT_DRIVE = new CANId(2, CANIVORE_BUS);
    public static final CANId FRONT_RIGHT_DRIVE = new CANId(5, CANIVORE_BUS);
    public static final CANId REAR_LEFT_DRIVE = new CANId(11, CANIVORE_BUS);
    public static final CANId REAR_RIGHT_DRIVE = new CANId(8, CANIVORE_BUS);

    public static final CANId FRONT_LEFT_STEER = new CANId(1);
    public static final CANId FRONT_RIGHT_STEER = new CANId(4);
    public static final CANId REAR_LEFT_STEER = new CANId(10);
    public static final CANId REAR_RIGHT_STEER = new CANId(7);

    public static final CANId FRONT_LEFT_STEER_ENCODER = new CANId(3, CANIVORE_BUS);
    public static final CANId FRONT_RIGHT_STEER_ENCODER = new CANId(6, CANIVORE_BUS);
    public static final CANId REAR_LEFT_STEER_ENCODER = new CANId(12, CANIVORE_BUS);
    public static final CANId REAR_RIGHT_STEER_ENCODER = new CANId(9, CANIVORE_BUS);

    private DeviceIds() {}
}
