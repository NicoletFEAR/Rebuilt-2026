package frc.robot.robots.tusk;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.robots.base.RobotRequest;
import frc.robot.subsystems.controller.ControllerRequest;
import frc.robot.subsystems.controller.ControllerRequestAutoLogged;
import frc.robot.subsystems.controller.ControllerState;

@AutoLog
public class TuskRequest extends RobotRequest<TuskState, TuskRequest> {
    public ControllerRequestAutoLogged DriverController = new ControllerRequestAutoLogged();
    public ControllerRequestAutoLogged OperatorController = new ControllerRequestAutoLogged();

    @Override
    public TuskRequest update(TuskState newState) {
        DriverController.update(newState.DriverController);
        OperatorController.update(newState.OperatorController);
        return this;
    }

    @Override
    public TuskRequest update(TuskRequest newRequest) {
        DriverController.update(newRequest.DriverController);
        OperatorController.update(newRequest.OperatorController);
        return this;
    }

    public TuskRequest updateDriverController(ControllerState newState) {
        DriverController.update(newState);
        return this;
    }

    public TuskRequest updateDriverController(ControllerRequest newRequest) {
        DriverController.update(newRequest);
        return this;
    }

    public TuskRequest updateOperatorController(ControllerState newState) {
        OperatorController.update(newState);
        return this;
    }

    public TuskRequest updateOperatorController(ControllerRequest newRequest) {
        OperatorController.update(newRequest);
        return this;
    }

    public TuskRequestAutoLogged toAutoLogged() {
        TuskRequestAutoLogged result = new TuskRequestAutoLogged();
        result.update(this);
        return result;
    }
}
