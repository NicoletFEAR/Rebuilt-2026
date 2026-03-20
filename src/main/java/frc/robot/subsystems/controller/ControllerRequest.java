package frc.robot.subsystems.controller;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.base.State;
import frc.robot.util.constraint.BooleanConstraintAutoLogged;
import frc.robot.util.constraint.DoubleConstraintAutoLogged;

@AutoLog
public class ControllerRequest extends State<ControllerRequest, ControllerName, ControllerIdentity> {
    public BooleanConstraintAutoLogged Circle = new BooleanConstraintAutoLogged();
    public BooleanConstraintAutoLogged Create = new BooleanConstraintAutoLogged();
    public BooleanConstraintAutoLogged Cross = new BooleanConstraintAutoLogged();
    public BooleanConstraintAutoLogged Down = new BooleanConstraintAutoLogged();
    public BooleanConstraintAutoLogged Left = new BooleanConstraintAutoLogged();
    public BooleanConstraintAutoLogged LeftBumper = new BooleanConstraintAutoLogged();
    public DoubleConstraintAutoLogged LeftRumbleStrength = new DoubleConstraintAutoLogged();
    public BooleanConstraintAutoLogged LeftStick = new BooleanConstraintAutoLogged();
    public DoubleConstraintAutoLogged LeftTrigger = new DoubleConstraintAutoLogged();
    public DoubleConstraintAutoLogged LeftX = new DoubleConstraintAutoLogged();
    public DoubleConstraintAutoLogged LeftY = new DoubleConstraintAutoLogged();
    public BooleanConstraintAutoLogged Options = new BooleanConstraintAutoLogged();
    public BooleanConstraintAutoLogged PlayStation = new BooleanConstraintAutoLogged();
    public BooleanConstraintAutoLogged Right = new BooleanConstraintAutoLogged();
    public BooleanConstraintAutoLogged RightBumper = new BooleanConstraintAutoLogged();
    public DoubleConstraintAutoLogged RightRumbleStrength = new DoubleConstraintAutoLogged();
    public BooleanConstraintAutoLogged RightStick = new BooleanConstraintAutoLogged();
    public DoubleConstraintAutoLogged RightTrigger = new DoubleConstraintAutoLogged();
    public DoubleConstraintAutoLogged RightX = new DoubleConstraintAutoLogged();
    public DoubleConstraintAutoLogged RightY = new DoubleConstraintAutoLogged();
    public BooleanConstraintAutoLogged Square = new BooleanConstraintAutoLogged();
    public BooleanConstraintAutoLogged Touchpad = new BooleanConstraintAutoLogged();
    public BooleanConstraintAutoLogged Triangle = new BooleanConstraintAutoLogged();
    public BooleanConstraintAutoLogged Up = new BooleanConstraintAutoLogged();

    public ControllerRequest update(ControllerState state) {
        Circle.set(state.Circle);
        Create.set(state.Create);
        Cross.set(state.Cross);
        CurrentIdentity = state.CurrentIdentity;
        Down.set(state.Down);
        Left.set(state.Left);
        LeftBumper.set(state.LeftBumper);
        LeftRumbleStrength.set(state.LeftRumbleStrength);
        LeftStick.set(state.LeftStick);
        LeftTrigger.set(state.LeftTrigger);
        LeftX.set(state.LeftX);
        LeftY.set(state.LeftY);
        Name = state.Name;
        Options.set(state.Options);
        PlayStation.set(state.PlayStation);
        ProperIdentity = state.ProperIdentity;
        Right.set(state.Right);
        RightBumper.set(state.RightBumper);
        RightRumbleStrength.set(state.RightRumbleStrength);
        RightStick.set(state.RightStick);
        RightTrigger.set(state.RightTrigger);
        RightX.set(state.RightX);
        RightY.set(state.RightY);
        return this;
    }

    @Override
    public ControllerRequest update(ControllerRequest newState) {
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

    public ControllerRequestAutoLogged toAutoLogged() {
        ControllerRequestAutoLogged result = new ControllerRequestAutoLogged();
        result.update(this);
        return result;
    }
}
