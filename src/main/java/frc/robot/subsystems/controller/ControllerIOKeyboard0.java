package frc.robot.subsystems.controller;

import edu.wpi.first.wpilibj.GenericHID;

public class ControllerIOKeyboard0 implements ControllerIO {
    private final GenericHID m_keyboard;

    private boolean m_l1;
    private boolean m_l2;
    private double m_leftX;
    private double m_leftY;
    private int m_pov;
    private boolean m_r1;
    private boolean m_r2;
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
        m_l1 = m_keyboard.getRawButton(2);
        m_l2 = m_keyboard.getRawButton(1);
        m_leftX = m_keyboard.getRawAxis(0);
        m_leftY = m_keyboard.getRawAxis(1);
        m_pov = m_keyboard.getPOV();
        m_r1 = m_keyboard.getRawButton(3);
        m_r2 = m_keyboard.getRawButton(4);
        m_rightX = m_keyboard.getRawAxis(2);
    }

    @Override
    public void updateInputs(ControllerIOInputs inputs) {
        inputs.Down = m_pov == 135 || m_pov == 180 || m_pov == 225;
        inputs.Left = m_pov == 225 || m_pov == 270 || m_pov == 315;
        inputs.L1 = m_l1;
        inputs.L2 = m_l2 ? 1.0 : 0.0;
        inputs.LeftX = m_leftX;
        inputs.LeftY = m_leftY;
        inputs.R1 = m_r1;
        inputs.R2 = m_r2 ? 1.0 : 0.0;
        inputs.Right = m_pov == 45 || m_pov == 90 || m_pov == 135;
        inputs.RightX = m_rightX;
        inputs.Up = m_pov == 0 || m_pov == 45 || m_pov == 315;
    }
}
