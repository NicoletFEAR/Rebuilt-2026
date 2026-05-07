package frc.robot.io.controller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ControllerIOPS4 extends ControllerIO {
    private PS4Controller controller;
    private double leftRumbleStrength = 0.0;
    private double rightRumbleStrength = 0.0;

    public ControllerIOPS4(int port) {
        super(port);
        controller = new PS4Controller(port);
    }

    @Override
    public void leftRumble(double strength) {
        leftRumbleStrength = strength;
        controller.setRumble(RumbleType.kLeftRumble, strength);
    }

    @Override
    public void rightRumble(double strength) {
        rightRumbleStrength = strength;
        controller.setRumble(RumbleType.kRightRumble, strength);
    }

    @Override
    public void rumble(double strength) {
        leftRumbleStrength = strength;
        rightRumbleStrength = strength;
        controller.setRumble(RumbleType.kBothRumble, strength);
    }

    @Override
    public ControllerState updateState() {
        int pov = controller.getPOV();
        state.Circle = controller.getCircleButton();
        state.Create = controller.getShareButton();
        state.Cross = controller.getCrossButton();
        state.CurrentIdentity = ControllerIdentity.PS4;
        state.Down = pov == 135 || pov == 180 || pov == 225;
        state.Left = pov == 225 || pov == 270 || pov == 315;
        state.LeftBumper = controller.getL1Button();
        state.LeftRumbleStrength = leftRumbleStrength;
        state.LeftStick = controller.getL3Button();
        state.LeftTrigger = controller.getL2Axis();
        state.LeftX = -controller.getLeftX();
        state.LeftY = -controller.getLeftY();
        state.Options = controller.getOptionsButton();
        state.PlayStation = controller.getPSButton();
        state.ProperIdentity = ControllerIdentity.getIdentity(DriverStation.getJoystickName(port));
        state.Right = pov == 45 || pov == 90 || pov == 135;
        state.RightBumper = controller.getR1Button();
        state.RightRumbleStrength = rightRumbleStrength;
        state.RightStick = controller.getR3Button();
        state.RightTrigger = controller.getR2Axis();
        state.RightX = -controller.getRightX();
        state.RightY = -controller.getRightY();
        state.Square = controller.getSquareButton();
        state.Touchpad = controller.getTouchpadButton();
        state.Triangle = controller.getTriangleButton();
        state.Up = pov == 0 || pov == 45 || pov == 315;
        return state;
    }
}
