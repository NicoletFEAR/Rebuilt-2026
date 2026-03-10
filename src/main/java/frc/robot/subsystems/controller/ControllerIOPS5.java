package frc.robot.subsystems.controller;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ControllerIOPS5 implements ControllerIO {
    private final PS5Controller m_controller;

    private boolean m_circle;
    private boolean m_create;
    private boolean m_cross;
    private boolean m_l1;
    private double m_l2;
    private boolean m_l3;
    private double m_leftX;
    private double m_leftY;
    private boolean m_options;
    private boolean m_playStation;
    private int m_pov;
    private boolean m_r1;
    private double m_r2;
    private boolean m_r3;
    private double m_rightX;
    private double m_rightY;
    private boolean m_square;
    private boolean m_touchpad;
    private boolean m_triangle;

    public ControllerIOPS5(int port) {
        m_controller = new PS5Controller(port);
    }

    @Override
    public void setLeftRumble(double strength) {
        m_controller.setRumble(RumbleType.kLeftRumble, strength);
    }

    @Override
    public void setRightRumble(double strength) {
        m_controller.setRumble(RumbleType.kRightRumble, strength);
    }

    @Override
    public void setRumble(double strength) {
        m_controller.setRumble(RumbleType.kBothRumble, strength);
    }

    @Override
    public void refreshData() {
        m_circle = m_controller.getCircleButton();
        m_create = m_controller.getCreateButton();
        m_cross = m_controller.getCrossButton();
        m_l1 = m_controller.getL1Button();
        m_l2 = m_controller.getL2Axis();
        m_l3 = m_controller.getL3Button();
        m_leftX = m_controller.getLeftX();
        m_leftY = m_controller.getLeftY();
        m_options = m_controller.getOptionsButton();
        m_playStation = m_controller.getPSButton();
        m_pov = m_controller.getPOV();
        m_r1 = m_controller.getR1Button();
        m_r2 = m_controller.getR2Axis();
        m_r3 = m_controller.getR3Button();
        m_rightX = m_controller.getRightX();
        m_rightY = m_controller.getRightY();
        m_square = m_controller.getSquareButton();
        m_touchpad = m_controller.getTouchpadButton();
        m_triangle = m_controller.getTriangleButton();
    }

    @Override
    public void updateInputs(ControllerIOInputs inputs) {
        inputs.Circle = m_circle;
        inputs.Create = m_create;
        inputs.Cross = m_cross;
        inputs.Down = m_pov == 135 || m_pov == 180 || m_pov == 225;
        inputs.L1 = m_l1;
        inputs.L2 = m_l2;
        inputs.L3 = m_l3;
        inputs.Left = m_pov == 225 || m_pov == 270 || m_pov == 315;
        inputs.LeftX = m_leftX;
        inputs.LeftY = m_leftY;
        inputs.Options = m_options;
        inputs.PlayStation = m_playStation;
        inputs.R1 = m_r1;
        inputs.R2 = m_r2;
        inputs.R3 = m_r3;
        inputs.Right = m_pov == 45 || m_pov == 90 || m_pov == 135;
        inputs.RightX = m_rightX;
        inputs.RightY = m_rightY;
        inputs.Square = m_square;
        inputs.Touchpad = m_touchpad;
        inputs.Triangle = m_triangle;
        inputs.Up = m_pov == 0 || m_pov == 45 || m_pov == 315;
    }
}
