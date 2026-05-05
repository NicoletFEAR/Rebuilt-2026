package frc.robot.robots.tusk;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.io.controller.ControllerState;
import frc.robot.io.controller.ControllerStateAutoLogged;
import frc.robot.robots.base.RobotState;
import frc.robot.subsystems.turn.TurnState;
import frc.robot.subsystems.turn.TurnStateAutoLogged;

@AutoLog
public class TuskState extends RobotState<TuskState> {
    public ControllerStateAutoLogged DriverController = new ControllerStateAutoLogged();
    public TurnStateAutoLogged FrontLeftTurn = new TurnStateAutoLogged();
    public TurnStateAutoLogged FrontRightTurn = new TurnStateAutoLogged();
    public ControllerStateAutoLogged OperatorController = new ControllerStateAutoLogged();
    public TurnStateAutoLogged RearLeftTurn = new TurnStateAutoLogged();
    public TurnStateAutoLogged RearRightTurn = new TurnStateAutoLogged();

    @Override
    public TuskState update(TuskState newState) {
        DriverController.update(newState.DriverController);
        FrontLeftTurn.update(newState.FrontLeftTurn);
        FrontRightTurn.update(newState.FrontRightTurn);
        OperatorController.update(newState.OperatorController);
        RearLeftTurn.update(newState.RearLeftTurn);
        RearRightTurn.update(newState.RearRightTurn);
        return this;
    }

    public TuskState updateDriverController(ControllerState newState) {
        DriverController.update(newState);
        return this;
    }

    public TuskState updateFrontLeftTurn(TurnState newState) {
        FrontLeftTurn.update(newState);
        return this;
    }

    public TuskState updateFrontRightTurn(TurnState newState) {
        FrontRightTurn.update(newState);
        return this;
    }

    public TuskState updateOperatorController(ControllerState newState) {
        OperatorController.update(newState);
        return this;
    }

    public TuskState updateRearLeftTurn(TurnState newState) {
        RearLeftTurn.update(newState);
        return this;
    }

    public TuskState updateRearRightTurn(TurnState newState) {
        RearRightTurn.update(newState);
        return this;
    }

    public TuskStateAutoLogged toAutoLogged() {
        TuskStateAutoLogged result = new TuskStateAutoLogged();
        result.update(this);
        return result;
    }
}
