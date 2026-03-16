package frc.robot.subsystems.controller;

import frc.robot.subsystems.controller.inputs.ControllerInputs;
import frc.robot.subsystems.controller.inputs.ControllerInputsNone;
import frc.robot.subsystems.controller.inputs.ControllerInputsPS4;
import frc.robot.subsystems.controller.inputs.ControllerInputsPS5;
import frc.robot.subsystems.controller.inputs.ControllerInputsXbox;

public enum ControllerIdentity {
    NONE("None"),
    KEYBOARD_0("Keyboard 0"),
    KEYBOARD_1("Keyboard 1"),
    KEYBOARD_2("Keyboard 2"),
    PS4("PS4"),
    PS5("PS5"),
    UNRECOGNIZED("Unrecognized"),
    XBOX("Xbox"),
    ;

    private final String m_name;

    ControllerIdentity(String name) {
        m_name = name;
    }

    @Override
    public String toString() {
        return m_name;
    }

    public ControllerInputs getInputs(ControllerName name, int port) {
        return switch (this) {
            case NONE -> new ControllerInputsNone(name, port);
            case PS4 -> new ControllerInputsPS4(name, port);
            case PS5 -> new ControllerInputsPS5(name, port);
            case UNRECOGNIZED -> new ControllerInputsNone(name, port);
            case XBOX -> new ControllerInputsXbox(name, port);
            default -> new ControllerInputsNone(name, port);
        };
    }

    public static ControllerIdentity getIdentity(String name) {
        return switch (name) {
            case "Keyboard 0" -> KEYBOARD_0;
            case "Keyboard 1" -> KEYBOARD_1;
            case "Keyboard 2" -> KEYBOARD_2;
            case "" -> NONE;
            case "Wireless Controller" -> PS4;
            case "DualSense Wireless Controller" -> PS5;
            case "Controller (Gamepad F310)", "Xbox Controller" -> XBOX;
            default -> UNRECOGNIZED;
        };
    }
}
