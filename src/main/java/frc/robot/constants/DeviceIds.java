package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

import frc.robot.util.CANId;

public final class DeviceIds {
    public static final int kDriverController = 0;
    public static final int kOperatorController = 1;

    public static final CANBus kRioBus = new CANBus("rio");
    public static final CANBus kCANivoreBus = new CANBus("*");

    public static final CANId kFrontLeftSteer = new CANId(1, kRioBus);
    public static final CANId kFrontRightSteer = new CANId(4, kRioBus);
    public static final CANId kRearLeftSteer = new CANId(10, kRioBus);
    public static final CANId kRearRightSteer = new CANId(7, kRioBus);

    public static final CANId kFrontLeftSteerAbsoluteEncoder = new CANId(3, kCANivoreBus);
    public static final CANId kFrontRightSteerAbsoluteEncoder = new CANId(6, kCANivoreBus);
    public static final CANId kRearLeftSteerAbsoluteEncoder = new CANId(12, kCANivoreBus);
    public static final CANId kRearRightSteerAbsoluteEncoder = new CANId(9, kCANivoreBus);

    private DeviceIds() {}
}
