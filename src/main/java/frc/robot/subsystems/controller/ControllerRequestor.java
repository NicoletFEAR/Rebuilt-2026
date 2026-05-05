package frc.robot.subsystems.controller;

import frc.robot.io.controller.ControllerIdentity;
import frc.robot.io.controller.ControllerState;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.Requestor;

public class ControllerRequestor extends Requestor<ControllerState, ControllerRequest, ControllerName, ControllerIdentity> {
    public ControllerRequestor(ControllerName name) {
        super(name);
        m_request = new ControllerRequest();
    }

    @Override
    public ControllerRequest requestTusk(TuskState fullState) {
        return switch (m_name) {
            case DRIVER -> m_request.update(fullState.DriverController);
            case OPERATOR -> m_request.update(fullState.OperatorController);
        };
    }
}
