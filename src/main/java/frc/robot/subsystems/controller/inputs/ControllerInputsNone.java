package frc.robot.subsystems.controller.inputs;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.controller.ControllerName;
import frc.robot.subsystems.controller.ControllerState;
import frc.robot.subsystems.controller.ControllerIdentity;

public class ControllerInputsNone extends ControllerInputs {
    private ControllerName m_name;

    private int m_port;

    private ControllerState m_state = new ControllerState();

    public ControllerInputsNone(ControllerName name, int port) {
        m_port = port;
        m_name = name;
    }

    @Override
    public void leftRumble(double strength) {}

    @Override
    public void rightRumble(double strength) {}

    @Override
    public void rumble(double strength) {}

    @Override
    public ControllerState updateState() {
        m_state.Circle = false;
        m_state.Create = false;
        m_state.Cross = false;
        m_state.CurrentType = ControllerIdentity.NONE;
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
        m_state.ProperType = ControllerIdentity.getIdentity(DriverStation.getJoystickName(m_port));
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
}
