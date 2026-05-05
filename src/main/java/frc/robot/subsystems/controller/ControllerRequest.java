package frc.robot.subsystems.controller;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.io.controller.ControllerIdentity;
import frc.robot.io.controller.ControllerState;
import frc.robot.subsystems.base.Request;
import frc.robot.util.constraint.BooleanConstraintAutoLogged;
import frc.robot.util.constraint.DoubleConstraintAutoLogged;

@AutoLog
public class ControllerRequest extends Request<ControllerState, ControllerRequest, ControllerIdentity> {
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

    @Override
    public ControllerRequest update(ControllerState newState) {
        Circle.set(newState.Circle);
        Create.set(newState.Create);
        Cross.set(newState.Cross);
        Down.set(newState.Down);
        Left.set(newState.Left);
        LeftBumper.set(newState.LeftBumper);
        LeftRumbleStrength.set(newState.LeftRumbleStrength);
        LeftStick.set(newState.LeftStick);
        LeftTrigger.set(newState.LeftTrigger);
        LeftX.set(newState.LeftX);
        LeftY.set(newState.LeftY);
        Options.set(newState.Options);
        PlayStation.set(newState.PlayStation);
        Right.set(newState.Right);
        RightBumper.set(newState.RightBumper);
        RightRumbleStrength.set(newState.RightRumbleStrength);
        RightStick.set(newState.RightStick);
        RightTrigger.set(newState.RightTrigger);
        RightX.set(newState.RightX);
        RightY.set(newState.RightY);
        return this;
    }

    @Override
    public ControllerRequest update(ControllerRequest newRequest) {
        Circle = newRequest.Circle;
        Create = newRequest.Create;
        Cross = newRequest.Cross;
        Down = newRequest.Down;
        Left = newRequest.Left;
        LeftBumper = newRequest.LeftBumper;
        LeftRumbleStrength = newRequest.LeftRumbleStrength;
        LeftStick = newRequest.LeftStick;
        LeftTrigger = newRequest.LeftTrigger;
        LeftX = newRequest.LeftX;
        LeftY = newRequest.LeftY;
        Options = newRequest.Options;
        PlayStation = newRequest.PlayStation;
        RightBumper = newRequest.RightBumper;
        RightRumbleStrength = newRequest.RightRumbleStrength;
        RightStick = newRequest.RightStick;
        RightTrigger = newRequest.RightTrigger;
        Right = newRequest.Right;
        RightX = newRequest.RightX;
        RightY = newRequest.RightY;
        Square = newRequest.Square;
        Touchpad = newRequest.Touchpad;
        Triangle = newRequest.Triangle;
        Up = newRequest.Up;
        return this;
    }

    public ControllerRequestAutoLogged toAutoLogged() {
        ControllerRequestAutoLogged result = new ControllerRequestAutoLogged();
        result.update(this);
        return result;
    }
}
