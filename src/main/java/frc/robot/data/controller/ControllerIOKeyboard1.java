package frc.robot.data.controller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

public class ControllerIOKeyboard1 extends ControllerIO {
    private int m_port;
    private final GenericHID m_keyboard;

    private boolean m_circle;
    private Type m_controllerType;
    private boolean m_cross;
    private double m_leftX;
    private double m_leftY;
    private boolean m_square;
    private boolean m_triangle;

    public ControllerIOKeyboard1(int port) {
        m_port = port;
        m_keyboard = new GenericHID(m_port);
    }

    @Override
    public void setRumble(double strength) {}

    @Override
    public void refreshData() {
        m_circle = m_keyboard.getRawButton(2);
        m_controllerType = getTypeFromName(DriverStation.getJoystickName(m_port));
        m_cross = m_keyboard.getRawButton(3);
        m_leftX = m_keyboard.getRawAxis(1);
        m_leftY = m_keyboard.getRawAxis(0);
        m_square = m_keyboard.getRawButton(4);
        m_triangle = m_keyboard.getRawButton(1);
    }

    @Override
    public void updateInputs(ControllerIOInputs inputs) {
        inputs.Circle = m_circle;
        inputs.ControllerType = m_controllerType;
        inputs.Create = false;
        inputs.Cross = m_cross;
        inputs.Down = false;
        inputs.LeftBumper = false;
        inputs.LeftTrigger = 0.0;
        inputs.LeftStick = false;
        inputs.Left = false;
        inputs.LeftX = m_leftX;
        inputs.LeftY = m_leftY;
        inputs.Options = false;
        inputs.PlayStation = false;
        inputs.RightBumper = false;
        inputs.RightTrigger = 0.0;
        inputs.RightStick = false;
        inputs.Right = false;
        inputs.RightX = 0.0;
        inputs.RightY = 0.0;
        inputs.Square = m_square;
        inputs.Touchpad = false;
        inputs.Triangle = m_triangle;
        inputs.Up = false;
    }
}
