package frc.robot.io.controller;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.io.base.IO;

public abstract class ControllerIO extends IO<ControllerState, ControllerIO, ControllerIdentity> {
    protected final int port;

    public ControllerIO(int port) {
        this.port = port;
        state = new ControllerState();
    }

    public void leftRumble(double strength) {}
    public void rightRumble(double strength) {}
    public void rumble(double strength) {}

    @Override
    public ControllerState updateState() {
        state.Circle = false;
        state.Create = false;
        state.Cross = false;
        state.CurrentIdentity = ControllerIdentity.NONE;
        state.Down = false;
        state.Left = false;
        state.LeftBumper = false;
        state.LeftRumbleStrength = 0.0;
        state.LeftStick = false;
        state.LeftTrigger = 0.0;
        state.LeftX = 0.0;
        state.LeftY = 0.0;
        state.Options = false;
        state.PlayStation = false;
        state.ProperIdentity = ControllerIdentity.getIdentity(DriverStation.getJoystickName(port));
        state.Right = false;
        state.RightBumper = false;
        state.RightRumbleStrength = 0.0;
        state.RightStick = false;
        state.RightTrigger = 0.0;
        state.RightX = 0.0;
        state.RightY = 0.0;
        state.Square = false;
        state.Touchpad = false;
        state.Triangle = false;
        state.Up = false;
        return state;
    }

    @Override
    public ControllerIO getProperIO() {
        return switch (state.ProperIdentity) {
            case NONE -> new ControllerIONone(port);
            case KEYBOARD_0 -> new ControllerIONone(port);
            case KEYBOARD_1 -> new ControllerIONone(port);
            case KEYBOARD_2 -> new ControllerIONone(port);
            case PS4 -> new ControllerIOPS4(port);
            case PS5 -> new ControllerIOPS5(port);
            case XBOX -> new ControllerIOXbox(port);
        };
    }
}
