package frc.robot.robots.hades;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.robots.base.RobotState;
import frc.robot.subsystems.controller.ControllerState;
import frc.robot.subsystems.controller.ControllerStateAutoLogged;

@AutoLog
public class HadesState extends RobotState<HadesState> {
    public ControllerStateAutoLogged ControllerState = new ControllerStateAutoLogged();

    @Override
    public HadesState update(HadesState newState) {
        ControllerState.update(newState.ControllerState);
        return this;
    }

    public HadesState updateControllerState(ControllerState newState) {
        ControllerState.update(newState);
        return this;
    }

    public HadesStateAutoLogged toAutoLogged() {
        HadesStateAutoLogged result = new HadesStateAutoLogged();
        result.update(this);
        return result;
    }
}
