package frc.robot.data.controller;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.architecture.SubsystemIO;

public abstract class ControllerIO implements SubsystemIO<ControllerIO.ControllerIOInputs> {
    @AutoLog
    public static class ControllerIOInputs {
        public boolean Circle = false;
        public Type ControllerType = Type.NONE;
        public boolean Create = false;
        public boolean Cross = false;
        public boolean Down = false;
        public boolean LeftBumper = false;
        public double LeftTrigger = 0.0;
        public boolean LeftStick = false;
        public boolean Left = false;
        public double LeftX = 0.0;
        public double LeftY = 0.0;
        public boolean Options = false;
        public boolean PlayStation = false;
        public boolean RightBumper = false;
        public double RightTrigger = 0.0;
        public boolean RightStick = false;
        public boolean Right = false;
        public double RightX = 0.0;
        public double RightY = 0.0;
        public boolean Square = false;
        public boolean Touchpad = false;
        public boolean Triangle = false;
        public boolean Up = false;
    }
    
    public abstract void setRumble(double strength);

    public static enum Type {
        KEYBOARD_0,
        KEYBOARD_1,
        KEYBOARD_2,
        NONE,
        PS_4,
        PS_5,
        UNRECOGNIZED,
        XBOX,
    }

    protected Type getTypeFromName(String name) {
        return switch (name) {
            case "Keyboard 0" -> Type.KEYBOARD_0;
            case "Keyboard 1" -> Type.KEYBOARD_1;
            case "Keyboard 2" -> Type.KEYBOARD_2;
            case "Wireless Controller" -> Type.PS_4;
            case "DualSense Wireless Controller" -> Type.PS_5;
            case "Controller (Gamepad F310)", "Xbox Controller" -> Type.XBOX;
            case "" -> Type.NONE;
            default -> Type.UNRECOGNIZED;
        };
    }
}
