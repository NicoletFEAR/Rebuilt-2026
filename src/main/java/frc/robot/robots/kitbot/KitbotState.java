package frc.robot.robots.kitbot;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.robots.base.RobotState;
import frc.robot.subsystems.controller.ControllerState;
import frc.robot.subsystems.controller.ControllerStateAutoLogged;
import frc.robot.subsystems.turn.TurnState;
import frc.robot.subsystems.turn.TurnStateAutoLogged;

@AutoLog
public class KitbotState extends RobotState<KitbotState> {
    public ControllerStateAutoLogged Controller = new ControllerStateAutoLogged();
    public TurnStateAutoLogged FrontLeftTurn = new TurnStateAutoLogged();
    public TurnStateAutoLogged FrontRightTurn = new TurnStateAutoLogged();
    public TurnStateAutoLogged RearLeftTurn = new TurnStateAutoLogged();
    public TurnStateAutoLogged RearRightTurn = new TurnStateAutoLogged();

    @Override
    public KitbotState update(KitbotState newState) {
        Controller.update(newState.Controller);
        FrontLeftTurn.update(newState.FrontLeftTurn);
        FrontRightTurn.update(newState.FrontRightTurn);
        RearLeftTurn.update(newState.RearLeftTurn);
        RearRightTurn.update(newState.RearRightTurn);
        return this;
    }

    public KitbotState updateController(ControllerState newState) {
        Controller.update(newState);
        return this;
    }

    public KitbotState updateFrontLeftTurn(TurnState newState) {
        FrontLeftTurn.update(newState);
        return this;
    }

    public KitbotState updateFrontRightTurn(TurnState newState) {
        FrontRightTurn.update(newState);
        return this;
    }

    public KitbotState updateRearLeftTurn(TurnState newState) {
        RearLeftTurn.update(newState);
        return this;
    }

    public KitbotState updateRearRightTurn(TurnState newState) {
        RearRightTurn.update(newState);
        return this;
    }

    public KitbotStateAutoLogged toAutoLogged() {
        KitbotStateAutoLogged result = new KitbotStateAutoLogged();
        result.update(this);
        return result;
    }
}
