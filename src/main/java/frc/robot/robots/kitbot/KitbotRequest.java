package frc.robot.robots.kitbot;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.robots.base.RobotRequest;
import frc.robot.subsystems.controller.ControllerRequest;
import frc.robot.subsystems.controller.ControllerRequestAutoLogged;
import frc.robot.subsystems.controller.ControllerState;
import frc.robot.subsystems.turn.TurnRequest;
import frc.robot.subsystems.turn.TurnRequestAutoLogged;
import frc.robot.subsystems.turn.TurnState;

@AutoLog
public class KitbotRequest extends RobotRequest<KitbotState, KitbotRequest> {
    public ControllerRequestAutoLogged Controller = new ControllerRequestAutoLogged();
    public TurnRequestAutoLogged FrontLeftTurn = new TurnRequestAutoLogged();
    public TurnRequestAutoLogged FrontRightTurn = new TurnRequestAutoLogged();
    public TurnRequestAutoLogged RearLeftTurn = new TurnRequestAutoLogged();
    public TurnRequestAutoLogged RearRightTurn = new TurnRequestAutoLogged();

    @Override
    public KitbotRequest update(KitbotState newState) {
        Controller.update(newState.Controller);
        FrontLeftTurn.update(newState.FrontLeftTurn);
        FrontRightTurn.update(newState.FrontRightTurn);
        RearLeftTurn.update(newState.RearLeftTurn);
        RearRightTurn.update(newState.RearRightTurn);
        return this;
    }

    @Override
    public KitbotRequest update(KitbotRequest newRequest) {
        Controller.update(newRequest.Controller);
        FrontLeftTurn.update(newRequest.FrontLeftTurn);
        FrontRightTurn.update(newRequest.FrontRightTurn);
        RearLeftTurn.update(newRequest.RearLeftTurn);
        RearRightTurn.update(newRequest.RearRightTurn);
        return this;
    }

    public KitbotRequest updateController(ControllerState newState) {
        Controller.update(newState);
        return this;
    }

    public KitbotRequest updateController(ControllerRequest newRequest) {
        Controller.update(newRequest);
        return this;
    }

    public KitbotRequest updateFrontLeftTurn(TurnState newState) {
        FrontLeftTurn.update(newState);
        return this;
    }

    public KitbotRequest updateFrontLeftTurn(TurnRequest newRequest) {
        FrontLeftTurn.update(newRequest);
        return this;
    }

    public KitbotRequest updateFrontRightTurn(TurnState newState) {
        FrontRightTurn.update(newState);
        return this;
    }

    public KitbotRequest updateFrontRightTurn(TurnRequest newRequest) {
        FrontRightTurn.update(newRequest);
        return this;
    }

    public KitbotRequest updateRearLeftTurn(TurnState newState) {
        RearLeftTurn.update(newState);
        return this;
    }

    public KitbotRequest updateRearLeftTurn(TurnRequest newRequest) {
        RearLeftTurn.update(newRequest);
        return this;
    }

    public KitbotRequest updateRearRightTurn(TurnState newState) {
        RearRightTurn.update(newState);
        return this;
    }

    public KitbotRequest updateRearRightTurn(TurnRequest newRequest) {
        RearRightTurn.update(newRequest);
        return this;
    }

    public KitbotRequestAutoLogged toAutoLogged() {
        KitbotRequestAutoLogged result = new KitbotRequestAutoLogged();
        result.update(this);
        return result;
    }
}
