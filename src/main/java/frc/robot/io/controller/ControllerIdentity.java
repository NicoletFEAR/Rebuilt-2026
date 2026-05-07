package frc.robot.io.controller;

public enum ControllerIdentity {
    NONE("None"),
    KEYBOARD_0("Keyboard 0"),
    KEYBOARD_1("Keyboard 1"),
    KEYBOARD_2("Keyboard 2"),
    PS4("PS4"),
    PS5("PS5"),
    XBOX("Xbox"),
    ;

    private final String name;

    ControllerIdentity(String name) {
        this.name = name;
    }

    @Override
    public String toString() {
        return name;
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
            default -> NONE;
        };
    }
}
