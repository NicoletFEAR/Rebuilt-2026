package frc.robot.subsystems.controller;

import frc.robot.constants.DeviceIds;

public enum ControllerName {
    DRIVER("Driver"),
    OPERATOR("Operator"),
    ;

    private final String m_name;

    ControllerName(String name) {
        m_name = name;
    }

    @Override
    public String toString() {
        return m_name;
    }

    public int getPort() {
        return switch (this) {
            case DRIVER -> DeviceIds.kDriverController;
            case OPERATOR -> DeviceIds.kOperatorController;
        };
    }
}
