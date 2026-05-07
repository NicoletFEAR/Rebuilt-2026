package frc.robot.io.joystick;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS5Controller;

public class PS5Joystick extends Joystick {
    private PS5Controller joystick;
    private double leftRumbleStrength = 0.0;
    private double rightRumbleStrength = 0.0;

    public PS5Joystick(int port) {
        super(port);
        joystick = new PS5Controller(port);
    }

    @Override
    public void leftRumble(double strength) {
        leftRumbleStrength = strength;
        joystick.setRumble(RumbleType.kLeftRumble, strength);
    }

    @Override
    public void rightRumble(double strength) {
        rightRumbleStrength = strength;
        joystick.setRumble(RumbleType.kRightRumble, strength);
    }

    @Override
    public void rumble(double strength) {
        leftRumbleStrength = strength;
        rightRumbleStrength = strength;
        joystick.setRumble(RumbleType.kBothRumble, strength);
    }

    @Override
    public JoystickState updateState() {
        int pov = joystick.getPOV();
        state.Circle = joystick.getCircleButton();
        state.Create = joystick.getCreateButton();
        state.Cross = joystick.getCrossButton();
        state.CurrentIdentity = JoystickIdentity.PS5;
        state.Down = pov == 135 || pov == 180 || pov == 225;
        state.Left = pov == 225 || pov == 270 || pov == 315;
        state.LeftBumper = joystick.getL1Button();
        state.LeftRumbleStrength = leftRumbleStrength;
        state.LeftStick = joystick.getL3Button();
        state.LeftTrigger = joystick.getL2Axis();
        state.LeftX = -joystick.getLeftX();
        state.LeftY = -joystick.getLeftY();
        state.Options = joystick.getOptionsButton();
        state.PlayStation = joystick.getPSButton();
        state.ProperIdentity = JoystickIdentity.getIdentity(DriverStation.getJoystickName(port));
        state.Right = pov == 45 || pov == 90 || pov == 135;
        state.RightBumper = joystick.getR1Button();
        state.RightRumbleStrength = rightRumbleStrength;
        state.RightStick = joystick.getR3Button();
        state.RightTrigger = joystick.getR2Axis();
        state.RightX = -joystick.getRightX();
        state.RightY = -joystick.getRightY();
        state.Square = joystick.getSquareButton();
        state.Touchpad = joystick.getTouchpadButton();
        state.Triangle = joystick.getTriangleButton();
        state.Up = pov == 0 || pov == 45 || pov == 315;
        return state;
    }
}
