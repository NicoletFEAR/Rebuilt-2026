package frc.robot.subsystems.controller;

import frc.robot.io.joystick.JoystickIdentity;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.Requestor;

public class ControllerRequestor extends Requestor<ControllerState, ControllerRequest, ControllerName, JoystickIdentity> {
    public ControllerRequestor(ControllerName name) {
        super(name);
        request = new ControllerRequest();
    }

    @Override
    public ControllerRequest request(TuskState fullState) {
        return switch (name) {
            case DRIVER -> request.update(fullState.DriverController);
            case OPERATOR -> request.update(fullState.OperatorController);
        };
    }
}
