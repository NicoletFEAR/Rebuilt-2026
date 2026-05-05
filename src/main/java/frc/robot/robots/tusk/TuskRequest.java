package frc.robot.robots.tusk;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.io.controller.ControllerState;
import frc.robot.robots.base.RobotRequest;
import frc.robot.subsystems.controller.ControllerRequest;
import frc.robot.subsystems.controller.ControllerRequestAutoLogged;
import frc.robot.subsystems.turn.TurnRequest;
import frc.robot.subsystems.turn.TurnRequestAutoLogged;
import frc.robot.subsystems.turn.TurnState;

@AutoLog
public class TuskRequest extends RobotRequest<TuskState, TuskRequest> {
    public ControllerRequestAutoLogged DriverController = new ControllerRequestAutoLogged();
    public TurnRequestAutoLogged FrontLeftTurn = new TurnRequestAutoLogged();
    public TurnRequestAutoLogged FrontRightTurn = new TurnRequestAutoLogged();
    public ControllerRequestAutoLogged OperatorController = new ControllerRequestAutoLogged();
    public TurnRequestAutoLogged RearLeftTurn = new TurnRequestAutoLogged();
    public TurnRequestAutoLogged RearRightTurn = new TurnRequestAutoLogged();

    @Override
    public TuskRequest update(TuskState newState) {
        DriverController.update(newState.DriverController);
        FrontLeftTurn.update(newState.FrontLeftTurn);
        FrontRightTurn.update(newState.FrontRightTurn);
        OperatorController.update(newState.OperatorController);
        RearLeftTurn.update(newState.RearLeftTurn);
        RearRightTurn.update(newState.RearRightTurn);
        return this;
    }

    @Override
    public TuskRequest update(TuskRequest newRequest) {
        DriverController.update(newRequest.DriverController);
        FrontLeftTurn.update(newRequest.FrontLeftTurn);
        FrontRightTurn.update(newRequest.FrontRightTurn);
        OperatorController.update(newRequest.OperatorController);
        RearLeftTurn.update(newRequest.RearLeftTurn);
        RearRightTurn.update(newRequest.RearRightTurn);
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

    public TuskRequest updateFrontLeftTurn(TurnState newState) {
        FrontLeftTurn.update(newState);
        return this;
    }

    public TuskRequest updateFrontLeftTurn(TurnRequest newRequest) {
        FrontLeftTurn.update(newRequest);
        return this;
    }

    public TuskRequest updateFrontRightTurn(TurnState newState) {
        FrontRightTurn.update(newState);
        return this;
    }

    public TuskRequest updateFrontRightTurn(TurnRequest newRequest) {
        FrontRightTurn.update(newRequest);
        return this;
    }

    public TuskRequest updateRearLeftTurn(TurnState newState) {
        RearLeftTurn.update(newState);
        return this;
    }

    public TuskRequest updateRearLeftTurn(TurnRequest newRequest) {
        RearLeftTurn.update(newRequest);
        return this;
    }

    public TuskRequest updateRearRightTurn(TurnState newState) {
        RearRightTurn.update(newState);
        return this;
    }

    public TuskRequest updateRearRightTurn(TurnRequest newRequest) {
        RearRightTurn.update(newRequest);
        return this;
    }

    public TuskRequestAutoLogged toAutoLogged() {
        TuskRequestAutoLogged result = new TuskRequestAutoLogged();
        result.update(this);
        return result;
    }
}
