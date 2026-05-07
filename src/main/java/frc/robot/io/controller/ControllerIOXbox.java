package frc.robot.io.controller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

public class ControllerIOXbox extends ControllerIO {
    private XboxController controller;
    private double leftRumbleStrength = 0.0;
    private double rightRumbleStrength = 0.0;

    public ControllerIOXbox(int port) {
        super(port);
        controller = new XboxController(port);
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
        state.Circle = controller.getBButton();
        state.Create = controller.getBackButton();
        state.Cross = controller.getAButton();
        state.CurrentIdentity = ControllerIdentity.XBOX;
        state.Down = pov == 135 || pov == 180 || pov == 225;
        state.Left = pov == 225 || pov == 270 || pov == 315;
        state.LeftBumper = controller.getLeftBumperButton();
        state.LeftRumbleStrength = leftRumbleStrength;
        state.LeftStick = controller.getLeftStickButton();
        state.LeftTrigger = controller.getLeftTriggerAxis();
        state.LeftX = -controller.getLeftX();
        state.LeftY = -controller.getLeftY();
        state.Options = controller.getStartButton();
        state.PlayStation = false;
        state.ProperIdentity = ControllerIdentity.getIdentity(DriverStation.getJoystickName(port));
        state.Right = pov == 45 || pov == 90 || pov == 135;
        state.RightBumper = controller.getRightBumperButton();
        state.RightRumbleStrength = rightRumbleStrength;
        state.RightStick = controller.getRightStickButton();
        state.RightTrigger = controller.getRightTriggerAxis();
        state.RightX = -controller.getRightX();
        state.RightY = -controller.getRightY();
        state.Square = controller.getXButton();
        state.Touchpad = false;
        state.Triangle = controller.getYButton();
        state.Up = pov == 0 || pov == 45 || pov == 315;
        return state;
    }
}
