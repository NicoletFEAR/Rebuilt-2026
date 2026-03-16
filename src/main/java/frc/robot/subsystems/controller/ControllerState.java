package frc.robot.subsystems.controller;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.base.State;

@AutoLog
public class ControllerState extends State<ControllerState, ControllerName, ControllerIdentity> {
    public boolean Circle = false;
    public boolean Create = false;
    public boolean Cross = false;
    public boolean Down = false;
    public boolean Left = false;
    public boolean LeftBumper = false;
    public double LeftRumbleStrength = 0.0;
    public boolean LeftStick = false;
    public double LeftTrigger = 0.0;
    public double LeftX = 0.0;
    public double LeftY = 0.0;
    public boolean Options = false;
    public boolean PlayStation = false;
    public boolean Right = false;
    public boolean RightBumper = false;
    public double RightRumbleStrength = 0.0;
    public boolean RightStick = false;
    public double RightTrigger = 0.0;
    public double RightX = 0.0;
    public double RightY = 0.0;
    public boolean Square = false;
    public boolean Touchpad = false;
    public boolean Triangle = false;
    public boolean Up = false;

    @Override
    public ControllerState update(ControllerState newState) {
        Circle = newState.Circle;
        Create = newState.Create;
        Cross = newState.Cross;
        CurrentIdentity = newState.CurrentIdentity;
        Down = newState.Down;
        Left = newState.Left;
        LeftBumper = newState.LeftBumper;
        LeftRumbleStrength = newState.LeftRumbleStrength;
        LeftStick = newState.LeftStick;
        LeftTrigger = newState.LeftTrigger;
        LeftX = newState.LeftX;
        LeftY = newState.LeftY;
        Options = newState.Options;
        PlayStation = newState.PlayStation;
        ProperIdentity = newState.ProperIdentity;
        Name = newState.Name;
        RightBumper = newState.RightBumper;
        RightRumbleStrength = newState.RightRumbleStrength;
        RightStick = newState.RightStick;
        RightTrigger = newState.RightTrigger;
        Right = newState.Right;
        RightX = newState.RightX;
        RightY = newState.RightY;
        Square = newState.Square;
        Touchpad = newState.Touchpad;
        Triangle = newState.Triangle;
        Up = newState.Up;
        return this;
    }

    public ControllerStateAutoLogged toAutoLogged() {
        ControllerStateAutoLogged result = new ControllerStateAutoLogged();
        result.update(this);
        return result;
    }
}
