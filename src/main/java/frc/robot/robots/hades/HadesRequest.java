package frc.robot.robots.hades;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.robots.base.RobotRequest;
import frc.robot.subsystems.controller.ControllerRequest;
import frc.robot.subsystems.controller.ControllerRequestAutoLogged;
import frc.robot.subsystems.controller.ControllerState;
import frc.robot.subsystems.turn.TurnRequest;
import frc.robot.subsystems.turn.TurnRequestAutoLogged;
import frc.robot.subsystems.turn.TurnState;

@AutoLog
public class HadesRequest extends RobotRequest<HadesState, HadesRequest> {
    public ControllerRequestAutoLogged Controller = new ControllerRequestAutoLogged();
    public TurnRequestAutoLogged FrontLeftTurn = new TurnRequestAutoLogged();
    public TurnRequestAutoLogged FrontRightTurn = new TurnRequestAutoLogged();
    public TurnRequestAutoLogged RearLeftTurn = new TurnRequestAutoLogged();
    public TurnRequestAutoLogged RearRightTurn = new TurnRequestAutoLogged();

    @Override
    public HadesRequest update(HadesState newState) {
        Controller.update(newState.Controller);
        FrontLeftTurn.update(newState.FrontLeftTurn);
        FrontRightTurn.update(newState.FrontRightTurn);
        RearLeftTurn.update(newState.RearLeftTurn);
        RearRightTurn.update(newState.RearRightTurn);
        return this;
    }

    @Override
    public HadesRequest update(HadesRequest newRequest) {
        Controller.update(newRequest.Controller);
        FrontLeftTurn.update(newRequest.FrontLeftTurn);
        FrontRightTurn.update(newRequest.FrontRightTurn);
        RearLeftTurn.update(newRequest.RearLeftTurn);
        RearRightTurn.update(newRequest.RearRightTurn);
        return this;
    }

    public HadesRequest updateController(ControllerState newState) {
        Controller.update(newState);
        return this;
    }

    public HadesRequest updateController(ControllerRequest newRequest) {
        Controller.update(newRequest);
        return this;
    }

    public HadesRequest updateFrontLeftTurn(TurnState newState) {
        FrontLeftTurn.update(newState);
        return this;
    }

    public HadesRequest updateFrontLeftTurn(TurnRequest newRequest) {
        FrontLeftTurn.update(newRequest);
        return this;
    }

    public HadesRequest updateFrontRightTurn(TurnState newState) {
        FrontRightTurn.update(newState);
        return this;
    }

    public HadesRequest updateFrontRightTurn(TurnRequest newRequest) {
        FrontRightTurn.update(newRequest);
        return this;
    }

    public HadesRequest updateRearLeftTurn(TurnState newState) {
        RearLeftTurn.update(newState);
        return this;
    }

    public HadesRequest updateRearLeftTurn(TurnRequest newRequest) {
        RearLeftTurn.update(newRequest);
        return this;
    }

    public HadesRequest updateRearRightTurn(TurnState newState) {
        RearRightTurn.update(newState);
        return this;
    }

    public HadesRequest updateRearRightTurn(TurnRequest newRequest) {
        RearRightTurn.update(newRequest);
        return this;
    }

    public HadesRequestAutoLogged toAutoLogged() {
        HadesRequestAutoLogged result = new HadesRequestAutoLogged();
        result.update(this);
        return result;
    }
}
