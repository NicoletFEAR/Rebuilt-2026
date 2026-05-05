package frc.robot.io.controller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS5Controller;

public class ControllerIOPS5 extends ControllerIO {
    private PS5Controller m_controller;
    private double m_leftRumbleStrength = 0.0;
    private double m_rightRumbleStrength = 0.0;

    public ControllerIOPS5(int port) {
        super(port);
        m_controller = new PS5Controller(m_port);
    }

    @Override
    public void leftRumble(double strength) {
        m_leftRumbleStrength = strength;
        m_controller.setRumble(RumbleType.kLeftRumble, strength);
    }

    @Override
    public void rightRumble(double strength) {
        m_rightRumbleStrength = strength;
        m_controller.setRumble(RumbleType.kRightRumble, strength);
    }

    @Override
    public void rumble(double strength) {
        m_leftRumbleStrength = strength;
        m_rightRumbleStrength = strength;
        m_controller.setRumble(RumbleType.kBothRumble, strength);
    }

    @Override
    public ControllerState updateState() {
        int pov = m_controller.getPOV();
        m_state.Circle = m_controller.getCircleButton();
        m_state.Create = m_controller.getCreateButton();
        m_state.Cross = m_controller.getCrossButton();
        m_state.CurrentIdentity = ControllerIdentity.PS5;
        m_state.Down = pov == 135 || pov == 180 || pov == 225;
        m_state.Left = pov == 225 || pov == 270 || pov == 315;
        m_state.LeftBumper = m_controller.getL1Button();
        m_state.LeftRumbleStrength = m_leftRumbleStrength;
        m_state.LeftStick = m_controller.getL3Button();
        m_state.LeftTrigger = m_controller.getL2Axis();
        m_state.LeftX = -m_controller.getLeftX();
        m_state.LeftY = -m_controller.getLeftY();
        m_state.Options = m_controller.getOptionsButton();
        m_state.PlayStation = m_controller.getPSButton();
        m_state.ProperIdentity = ControllerIdentity.getIdentity(DriverStation.getJoystickName(m_port));
        m_state.Right = pov == 45 || pov == 90 || pov == 135;
        m_state.RightBumper = m_controller.getR1Button();
        m_state.RightRumbleStrength = m_rightRumbleStrength;
        m_state.RightStick = m_controller.getR3Button();
        m_state.RightTrigger = m_controller.getR2Axis();
        m_state.RightX = -m_controller.getRightX();
        m_state.RightY = -m_controller.getRightY();
        m_state.Square = m_controller.getSquareButton();
        m_state.Touchpad = m_controller.getTouchpadButton();
        m_state.Triangle = m_controller.getTriangleButton();
        m_state.Up = pov == 0 || pov == 45 || pov == 315;
        return m_state;
    }
}
