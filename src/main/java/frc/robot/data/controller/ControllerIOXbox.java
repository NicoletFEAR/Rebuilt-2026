package frc.robot.data.controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ControllerIOXbox implements ControllerIO {
    private final XboxController m_controller;

    private boolean m_circle;
    private boolean m_create;
    private boolean m_cross;
    private boolean m_l1;
    private double m_l2;
    private boolean m_l3;
    private double m_leftX;
    private double m_leftY;
    private boolean m_options;
    private int m_pov;
    private boolean m_r1;
    private double m_r2;
    private boolean m_r3;
    private double m_rightX;
    private double m_rightY;
    private boolean m_square;
    private boolean m_triangle;

    public ControllerIOXbox(int port) {
        m_controller = new XboxController(port);
    }

    @Override
    public void setRumble(double strength) {
        m_controller.setRumble(RumbleType.kBothRumble, strength);
    }

    @Override
    public void refreshData() {
        m_circle = m_controller.getBButton();
        m_create = m_controller.getBackButton();
        m_cross = m_controller.getAButton();
        m_l1 = m_controller.getLeftBumperButton();
        m_l2 = m_controller.getLeftTriggerAxis();
        m_l3 = m_controller.getLeftStickButton();
        m_leftX = m_controller.getLeftX();
        m_leftY = m_controller.getLeftY();
        m_options = m_controller.getStartButton();
        m_pov = m_controller.getPOV();
        m_r1 = m_controller.getRightBumperButton();
        m_r2 = m_controller.getRightTriggerAxis();
        m_r3 = m_controller.getRightStickButton();
        m_rightX = m_controller.getRightX();
        m_rightY = m_controller.getRightY();
        m_square = m_controller.getXButton();
        m_triangle = m_controller.getYButton();
    }

    @Override
    public void updateInputs(ControllerIOInputs inputs) {
        inputs.Circle = m_circle;
        inputs.Create = m_create;
        inputs.Cross = m_cross;
        inputs.Down = m_pov == 135 || m_pov == 180 || m_pov == 225;
        inputs.LeftBumper = m_l1;
        inputs.LeftTrigger = m_l2;
        inputs.LeftStick = m_l3;
        inputs.Left = m_pov == 225 || m_pov == 270 || m_pov == 315;
        inputs.LeftX = m_leftX;
        inputs.LeftY = m_leftY;
        inputs.Options = m_options;
        inputs.PlayStation = false;
        inputs.RightBumper = m_r1;
        inputs.RightTrigger = m_r2;
        inputs.RightStick = m_r3;
        inputs.Right = m_pov == 45 || m_pov == 90 || m_pov == 135;
        inputs.RightX = m_rightX;
        inputs.RightY = m_rightY;
        inputs.Square = m_square;
        inputs.Touchpad = false;
        inputs.Triangle = m_triangle;
        inputs.Up = m_pov == 0 || m_pov == 45 || m_pov == 315;
    }
}
