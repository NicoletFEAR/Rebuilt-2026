package frc.robot.subsystems.controller;

import frc.robot.constants.DeviceIds;

public enum ControllerName {
    DRIVER("Driver"),
    OPERATOR("Operator"),
    ;

    private final String name;

    ControllerName(String name) {
        this.name = name;
    }

    @Override
    public String toString() {
        return name;
    }

    public int getPort() {
        return switch (this) {
            case DRIVER -> DeviceIds.DRIVER_CONTROLLER;
            case OPERATOR -> DeviceIds.OPERATOR_CONTROLLER;
        };
    }
}
