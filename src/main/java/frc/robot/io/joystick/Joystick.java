package frc.robot.io.joystick;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.io.base.IO;

public abstract class Joystick extends IO<JoystickState, Joystick, JoystickIdentity> {
    protected final int port;

    public Joystick(int port) {
        this.port = port;
        state = new JoystickState();
    }

    public void leftRumble(double strength) {}
    public void rightRumble(double strength) {}
    public void rumble(double strength) {}

    @Override
    public JoystickState updateState() {
        state.Circle = false;
        state.Create = false;
        state.Cross = false;
        state.CurrentIdentity = JoystickIdentity.NONE;
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
        state.ProperIdentity = JoystickIdentity.getIdentity(DriverStation.getJoystickName(port));
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
    public Joystick getProperIO() {
        return switch (state.ProperIdentity) {
            case NONE -> new EmptyJoystick(port);
            case KEYBOARD_0 -> new EmptyJoystick(port);
            case KEYBOARD_1 -> new EmptyJoystick(port);
            case KEYBOARD_2 -> new EmptyJoystick(port);
            case PS4 -> new PS4Joystick(port);
            case PS5 -> new PS5Joystick(port);
            case XBOX -> new XboxJoystick(port);
        };
    }
}
