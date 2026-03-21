package frc.robot.util;

import com.ctre.phoenix6.CANBus;

public class CANId {
    private final CANBus m_bus;
    private final int m_device;

    public CANId(int device, CANBus bus) {
        m_bus = bus;
        m_device = device;
    }

    @Override
    public String toString() {
        return String.format("%d on bus %s", m_device, m_bus.getName());
    }

    public CANBus getBus() {
        return m_bus;
    }

    public int getDevice() {
        return m_device;
    }
}
