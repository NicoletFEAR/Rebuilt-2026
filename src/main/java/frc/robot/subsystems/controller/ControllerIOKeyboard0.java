package frc.robot.subsystems.controller;

import edu.wpi.first.wpilibj.GenericHID;

public class ControllerIOKeyboard0 implements ControllerIO {
    private final GenericHID m_keyboard;

    private boolean m_leftBumper;
    private boolean m_leftTrigger;
    private double m_leftX;
    private double m_leftY;
    private int m_pov;
    private boolean m_rightBumper;
    private boolean m_rightTrigger;
    private double m_rightX;

    public ControllerIOKeyboard0(int port) {
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
        m_leftBumper = m_keyboard.getRawButton(2);
        m_leftTrigger = m_keyboard.getRawButton(1);
        m_leftX = m_keyboard.getRawAxis(0);
        m_leftY = m_keyboard.getRawAxis(1);
        m_pov = m_keyboard.getPOV();
        m_rightBumper = m_keyboard.getRawButton(3);
        m_rightTrigger = m_keyboard.getRawButton(4);
        m_rightX = m_keyboard.getRawAxis(2);
    }

    @Override
    public void updateInputs(ControllerIOInputs inputs) {
        inputs.Circle = false;
        inputs.Create = false;
        inputs.Cross = false;
        inputs.Down = m_pov == 135 || m_pov == 180 || m_pov == 225;
        inputs.LeftBumper = m_leftBumper;
        inputs.LeftTrigger = m_leftTrigger ? 1.0 : 0.0;
        inputs.LeftStick = false;
        inputs.Left = m_pov == 225 || m_pov == 270 || m_pov == 315;
        inputs.LeftX = m_leftX;
        inputs.LeftY = m_leftY;
        inputs.Options = false;
        inputs.PlayStation = false;
        inputs.RightBumper = m_rightBumper;
        inputs.RightTrigger = m_rightTrigger ? 1.0 : 0.0;
        inputs.RightStick = false;
        inputs.Right = m_pov == 45 || m_pov == 90 || m_pov == 135;
        inputs.RightX = m_rightX;
        inputs.RightY = 0.0;
        inputs.Square = false;
        inputs.Touchpad = false;
        inputs.Triangle = false;
        inputs.Up = m_pov == 0 || m_pov == 45 || m_pov == 315;
    }
}
