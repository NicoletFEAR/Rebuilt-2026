package frc.robot.robots.hades;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.robots.base.RobotState;
import frc.robot.subsystems.controller.ControllerState;
import frc.robot.subsystems.controller.ControllerStateAutoLogged;
import frc.robot.subsystems.turn.TurnState;
import frc.robot.subsystems.turn.TurnStateAutoLogged;

@AutoLog
public class HadesState extends RobotState<HadesState> {
    public ControllerStateAutoLogged Controller = new ControllerStateAutoLogged();
    public TurnStateAutoLogged FrontLeftTurn = new TurnStateAutoLogged();
    public TurnStateAutoLogged FrontRightTurn = new TurnStateAutoLogged();
    public TurnStateAutoLogged RearLeftTurn = new TurnStateAutoLogged();
    public TurnStateAutoLogged RearRightTurn = new TurnStateAutoLogged();

    @Override
    public HadesState update(HadesState newState) {
        Controller.update(newState.Controller);
        FrontLeftTurn.update(newState.FrontLeftTurn);
        FrontRightTurn.update(newState.FrontRightTurn);
        RearLeftTurn.update(newState.RearLeftTurn);
        RearRightTurn.update(newState.RearRightTurn);
        return this;
    }

    public HadesState updateController(ControllerState newState) {
        Controller.update(newState);
        return this;
    }

    public HadesState updateFrontLeftTurn(TurnState newState) {
        FrontLeftTurn.update(newState);
        return this;
    }

    public HadesState updateFrontRightTurn(TurnState newState) {
        FrontRightTurn.update(newState);
        return this;
    }

    public HadesState updateRearLeftTurn(TurnState newState) {
        RearLeftTurn.update(newState);
        return this;
    }

    public HadesState updateRearRightTurn(TurnState newState) {
        RearRightTurn.update(newState);
        return this;
    }

    public HadesStateAutoLogged toAutoLogged() {
        HadesStateAutoLogged result = new HadesStateAutoLogged();
        result.update(this);
        return result;
    }
}
