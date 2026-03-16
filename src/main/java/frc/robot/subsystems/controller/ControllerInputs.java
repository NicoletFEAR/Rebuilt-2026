package frc.robot.subsystems.controller;

import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.subsystems.base.Inputs;

public class ControllerInputs extends Inputs<ControllerState> {
    private PS5Controller m_controller;
    private double m_leftRumbleStrength = 0.0;
    private ControllerName m_name;
    private double m_rightRumbleStrength = 0.0;

    private ControllerState m_state = new ControllerState();

    public ControllerInputs(int port, ControllerName name) {
        m_controller = new PS5Controller(port);
        m_name = name;
    }

    @Override
    public ControllerState updateState() {
        int pov = m_controller.getPOV();

        m_state.Circle = m_controller.getCircleButton();
        m_state.Create = m_controller.getCreateButton();
        m_state.Cross = m_controller.getCrossButton();
        m_state.Down = pov == 135 || pov == 180 || pov == 225;
        m_state.Left = pov == 225 || pov == 270 || pov == 315;
        m_state.LeftBumper = m_controller.getL1Button();
        m_state.LeftRumbleStrength = m_leftRumbleStrength;
        m_state.LeftStick = m_controller.getL3Button();
        m_state.LeftTrigger = m_controller.getL2Axis();
        m_state.LeftX = m_controller.getLeftX();
        m_state.LeftY = m_controller.getLeftY();
        m_state.Name = m_name;
        m_state.Options = m_controller.getOptionsButton();
        m_state.PlayStation = m_controller.getPSButton();
        m_state.Right = pov == 45 || pov == 90 || pov == 135;
        m_state.RightBumper = m_controller.getR1Button();
        m_state.RightRumbleStrength = m_rightRumbleStrength;
        m_state.RightStick = m_controller.getR3Button();
        m_state.RightTrigger = m_controller.getR2Axis();
        m_state.RightX = m_controller.getRightX();
        m_state.RightY = m_controller.getRightY();
        m_state.Square = m_controller.getSquareButton();
        m_state.Touchpad = m_controller.getTouchpadButton();
        m_state.Triangle = m_controller.getTriangleButton();
        m_state.Up = pov == 0 || pov == 45 || pov == 315;
        return m_state;
    }
}
