package frc.robot.subsystems.controller;

import edu.wpi.first.wpilibj.GenericHID;

public class ControllerIOKeyboard2 implements ControllerIO {
    private final GenericHID m_keyboard;

    private boolean m_create;
    private boolean m_leftBumper;
    private boolean m_leftTrigger;
    private double m_leftX;
    private double m_leftY;
    private boolean m_options;
    private boolean m_rightBumper;
    private boolean m_rightTrigger;

    public ControllerIOKeyboard2(int port) {
        m_keyboard = new GenericHID(port);
    }

    @Override
    public void setLeftRumble(double strength) {}

    @Override
    public void setRightRumble(double strength) {}

    @Override
    public void setRumble(double strength) {}

    @Override
    public void refreshData() {
        m_create = m_keyboard.getRawButton(3);
        m_leftBumper = m_keyboard.getRawButton(5);
        m_leftTrigger = m_keyboard.getRawButton(2);
        m_leftX = m_keyboard.getRawAxis(0);
        m_leftY = m_keyboard.getRawAxis(1);
        m_options = m_keyboard.getRawButton(6);
        m_rightBumper = m_keyboard.getRawButton(1);
        m_rightTrigger = m_keyboard.getRawButton(4);
    }

    @Override
    public void updateInputs(ControllerIOInputs inputs) {
        inputs.Circle = false;
        inputs.Create = m_create;
        inputs.Cross = false;
        inputs.Down = false;
        inputs.LeftBumper = m_leftBumper;
        inputs.LeftTrigger = m_leftTrigger ? 1.0 : 0.0;
        inputs.LeftStick = false;
        inputs.LeftX = m_leftX;
        inputs.LeftY = m_leftY;
        inputs.Options = m_options;
        inputs.PlayStation = false;
        inputs.RightBumper = m_rightBumper;
        inputs.RightTrigger = m_rightTrigger ? 1.0 : 0.0;
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
