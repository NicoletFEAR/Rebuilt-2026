package frc.robot.io.controller;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.io.base.IO;

public abstract class ControllerIO extends IO<ControllerState, ControllerIO, ControllerName, ControllerIdentity> {
    protected final int m_port;

    public ControllerIO(ControllerName name) {
        super(name);
        m_port = m_name.getPort();
        m_state = new ControllerState();
    }

    public void leftRumble(double strength) {}

    public void rightRumble(double strength) {}

    public void rumble(double strength) {}

    @Override
    public ControllerState updateState() {
        m_state.Circle = false;
        m_state.Create = false;
        m_state.Cross = false;
        m_state.CurrentIdentity = ControllerIdentity.NONE;
        m_state.Down = false;
        m_state.Left = false;
        m_state.LeftBumper = false;
        m_state.LeftRumbleStrength = 0.0;
        m_state.LeftStick = false;
        m_state.LeftTrigger = 0.0;
        m_state.LeftX = 0.0;
        m_state.LeftY = 0.0;
        m_state.Name = m_name;
        m_state.Options = false;
        m_state.PlayStation = false;
        m_state.ProperIdentity = ControllerIdentity.getIdentity(DriverStation.getJoystickName(m_port));
        m_state.Right = false;
        m_state.RightBumper = false;
        m_state.RightRumbleStrength = 0.0;
        m_state.RightStick = false;
        m_state.RightTrigger = 0.0;
        m_state.RightX = 0.0;
        m_state.RightY = 0.0;
        m_state.Square = false;
        m_state.Touchpad = false;
        m_state.Triangle = false;
        m_state.Up = false;
        return m_state;
    }

    @Override
    public ControllerIO getProperIO() {
        return switch (m_state.ProperIdentity) {
            case NONE -> new ControllerIONone(m_name);
            case KEYBOARD_0 -> new ControllerIONone(m_name);
            case KEYBOARD_1 -> new ControllerIONone(m_name);
            case KEYBOARD_2 -> new ControllerIONone(m_name);
            case PS4 -> new ControllerIOPS4(m_name);
            case PS5 -> new ControllerIOPS5(m_name);
            case XBOX -> new ControllerIOXbox(m_name);
        };
    }
}
