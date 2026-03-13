package frc.robot.data.controller;

import edu.wpi.first.wpilibj.DriverStation;

public class ControllerIONone extends ControllerIO {
    private int m_port;
    
    private Type m_controllerType;

    public ControllerIONone(int port) {
        m_port = port;
    }

    @Override
    public void setRumble(double strength) {}

    @Override
    public void refreshData() {
        m_controllerType = getTypeFromName(DriverStation.getJoystickName(m_port));
    }

    @Override
    public void updateInputs(ControllerIOInputs inputs) {
        inputs.Circle = false;
        inputs.ControllerType = m_controllerType;
        inputs.Create = false;
        inputs.Cross = false;
        inputs.Down = false;
        inputs.LeftBumper = false;
        inputs.LeftTrigger = 0.0;
        inputs.LeftStick = false;
        inputs.Left = false;
        inputs.LeftX = 0.0;
        inputs.LeftY = 0.0;
        inputs.Options = false;
        inputs.PlayStation = false;
        inputs.RightBumper = false;
        inputs.RightTrigger = 0.0;
        inputs.RightStick = false;
        inputs.Right = false;
        inputs.RightX = 0.0;
        inputs.RightY = 0.0;
        inputs.Square = false;
        inputs.Touchpad = false;
        inputs.Triangle = false;
        inputs.Up = false;
    }
}
