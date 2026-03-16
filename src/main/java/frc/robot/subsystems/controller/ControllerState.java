package frc.robot.subsystems.controller;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.base.State;

@AutoLog
public class ControllerState extends State<ControllerState> {
    public boolean Circle = false;
    public boolean Create = false;
    public boolean Cross = false;
    public ControllerIdentity CurrentType = ControllerIdentity.NONE;
    public boolean Down = false;
    public boolean Left = false;
    public boolean LeftBumper = false;
    public double LeftRumbleStrength = 0.0;
    public boolean LeftStick = false;
    public double LeftTrigger = 0.0;
    public double LeftX = 0.0;
    public double LeftY = 0.0;
    public ControllerName Name = ControllerName.DRIVER;
    public boolean Options = false;
    public boolean PlayStation = false;
    public ControllerIdentity ProperType = ControllerIdentity.NONE;
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
        this.Circle = newState.Circle;
        this.Create = newState.Create;
        this.Cross = newState.Cross;
        this.CurrentType = newState.CurrentType;
        this.Down = newState.Down;
        this.Left = newState.Left;
        this.LeftBumper = newState.LeftBumper;
        this.LeftRumbleStrength = newState.LeftRumbleStrength;
        this.LeftStick = newState.LeftStick;
        this.LeftTrigger = newState.LeftTrigger;
        this.LeftX = newState.LeftX;
        this.LeftY = newState.LeftY;
        this.Options = newState.Options;
        this.PlayStation = newState.PlayStation;
        this.ProperType = newState.ProperType;
        this.Name = newState.Name;
        this.RightBumper = newState.RightBumper;
        this.RightRumbleStrength = newState.RightRumbleStrength;
        this.RightStick = newState.RightStick;
        this.RightTrigger = newState.RightTrigger;
        this.Right = newState.Right;
        this.RightX = newState.RightX;
        this.RightY = newState.RightY;
        this.Square = newState.Square;
        this.Touchpad = newState.Touchpad;
        this.Triangle = newState.Triangle;
        this.Up = newState.Up;
        return this;
    }

    public ControllerStateAutoLogged toAutoLogged() {
        ControllerStateAutoLogged result = new ControllerStateAutoLogged();
        result.update(this);
        return result;
    }
}
