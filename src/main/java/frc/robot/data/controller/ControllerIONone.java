package frc.robot.data.controller;

public class ControllerIONone implements ControllerIO {
    @Override
    public void setLeftRumble(double strength) {}

    @Override
    public void setRightRumble(double strength) {}

    @Override
    public void setRumble(double strength) {}

    @Override
    public void refreshData() {}

    @Override
    public void updateInputs(ControllerIOInputs inputs) {
        inputs.Circle = false;
        inputs.Create = false;
        inputs.Cross = false;
        inputs.Down = false;
        inputs.LeftBumper = false;
        inputs.LeftTrigger = 0.0;
        inputs.LeftStick = false;
        inputs.Left = false;
        inputs.LeftX = 0.0;
        inputs.LeftY = 0.0;
        inputs.Options = false;
        inputs.PlayStation = false;
        inputs.RightBumper = false;
        inputs.RightTrigger = 0.0;
        inputs.RightStick = false;
        inputs.Right = false;
        inputs.RightX = 0.0;
        inputs.RightY = 0.0;
        inputs.Square = false;
        inputs.Touchpad = false;
        inputs.Triangle = false;
        inputs.Up = false;
    }
}
