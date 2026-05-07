package frc.robot.io.joystick;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

public class XboxJoystick extends Joystick {
    private XboxController joystick;
    private double leftRumbleStrength = 0.0;
    private double rightRumbleStrength = 0.0;

    public XboxJoystick(int port) {
        super(port);
        joystick = new XboxController(port);
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
        state.Circle = joystick.getBButton();
        state.Create = joystick.getBackButton();
        state.Cross = joystick.getAButton();
        state.CurrentIdentity = JoystickIdentity.XBOX;
        state.Down = pov == 135 || pov == 180 || pov == 225;
        state.Left = pov == 225 || pov == 270 || pov == 315;
        state.LeftBumper = joystick.getLeftBumperButton();
        state.LeftRumbleStrength = leftRumbleStrength;
        state.LeftStick = joystick.getLeftStickButton();
        state.LeftTrigger = joystick.getLeftTriggerAxis();
        state.LeftX = -joystick.getLeftX();
        state.LeftY = -joystick.getLeftY();
        state.Options = joystick.getStartButton();
        state.PlayStation = false;
        state.ProperIdentity = JoystickIdentity.getIdentity(DriverStation.getJoystickName(port));
        state.Right = pov == 45 || pov == 90 || pov == 135;
        state.RightBumper = joystick.getRightBumperButton();
        state.RightRumbleStrength = rightRumbleStrength;
        state.RightStick = joystick.getRightStickButton();
        state.RightTrigger = joystick.getRightTriggerAxis();
        state.RightX = -joystick.getRightX();
        state.RightY = -joystick.getRightY();
        state.Square = joystick.getXButton();
        state.Touchpad = false;
        state.Triangle = joystick.getYButton();
        state.Up = pov == 0 || pov == 45 || pov == 315;
        return state;
    }
}
