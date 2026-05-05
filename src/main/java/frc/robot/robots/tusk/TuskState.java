package frc.robot.robots.tusk;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.io.controller.ControllerState;
import frc.robot.io.controller.ControllerStateAutoLogged;
import frc.robot.robots.base.RobotState;

@AutoLog
public class TuskState extends RobotState<TuskState> {
    public ControllerStateAutoLogged DriverController = new ControllerStateAutoLogged();
    public ControllerStateAutoLogged OperatorController = new ControllerStateAutoLogged();

    @Override
    public TuskState update(TuskState newState) {
        DriverController.update(newState.DriverController);
        OperatorController.update(newState.OperatorController);
        return this;
    }

    public TuskState updateDriverController(ControllerState newState) {
        DriverController.update(newState);
        return this;
    }

    public TuskState updateOperatorController(ControllerState newState) {
        OperatorController.update(newState);
        return this;
    }

    public TuskStateAutoLogged toAutoLogged() {
        TuskStateAutoLogged result = new TuskStateAutoLogged();
        result.update(this);
        return result;
    }
}
