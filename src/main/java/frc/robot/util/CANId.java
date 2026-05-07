package frc.robot.util;

import com.ctre.phoenix6.CANBus;

import frc.robot.constants.DeviceIds;

public record CANId(int device, CANBus bus) {
    public CANId(int device) {
        this(device, DeviceIds.RIO_BUS);
    }
}
