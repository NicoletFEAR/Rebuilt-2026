package frc.robot.subsystems.controller;

import frc.robot.subsystems.base.Inputs;
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

    private String m_name;

    ControllerIdentity(String name) {
        m_name = name;
    }

    public String toString() {
        return m_name;
    }

    public Inputs<ControllerState> getInputs(ControllerName name, int port) {
        return switch (this) {
            case NONE -> new ControllerInputsNone(port, name);
            case PS4 -> new ControllerInputsPS4(port, name);
            case PS5 -> new ControllerInputsPS5(port, name);
            case UNRECOGNIZED -> new ControllerInputsNone(port, name);
            case XBOX -> new ControllerInputsXbox(port, name);
            default -> new ControllerInputsNone(port, name);
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
