package frc.robot.subsystems.controllers;

import edu.wpi.first.wpilibj.GenericHID;

public class ControllerIOKeyboard2 implements ControllerIO {
    private final GenericHID m_keyboard;

    private boolean m_create;
    private boolean m_l1;
    private boolean m_l2;
    private double m_leftX;
    private double m_leftY;
    private boolean m_options;
    private boolean m_r1;
    private boolean m_r2;

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
    public void refreshInputs() {
        m_create = m_keyboard.getRawButton(3);
        m_l1 = m_keyboard.getRawButton(5);
        m_l2 = m_keyboard.getRawButton(2);
        m_leftX = m_keyboard.getRawAxis(0);
        m_leftY = m_keyboard.getRawAxis(1);
        m_options = m_keyboard.getRawButton(6);
        m_r1 = m_keyboard.getRawButton(1);
        m_r2 = m_keyboard.getRawButton(4);
    }

    @Override
    public void updateInputs(ControllerIOInputs inputs) {
        inputs.Create = m_create;
        inputs.L1 = m_l1;
        inputs.L2 = m_l2 ? 1.0 : 0.0;
        inputs.LeftX = m_leftX;
        inputs.LeftY = m_leftY;
        inputs.Options = m_options;
        inputs.R1 = m_r1;
        inputs.R2 = m_r2 ? 1.0 : 0.0;
    }
}
