package frc.robot.robots.kitbot;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.robots.base.RobotState;
import frc.robot.subsystems.controller.ControllerState;
import frc.robot.subsystems.controller.ControllerStateAutoLogged;

@AutoLog
public class KitbotState extends RobotState<KitbotState> {
    public ControllerStateAutoLogged ControllerState = new ControllerStateAutoLogged();

    @Override
    public KitbotState update(KitbotState newState) {
        ControllerState.update(newState.ControllerState);
        return this;
    }

    public KitbotState updateControllerState(ControllerState newState) {
        ControllerState.update(newState);
        return this;
    }

    public KitbotStateAutoLogged toAutoLogged() {
        KitbotStateAutoLogged result = new KitbotStateAutoLogged();
        result.update(this);
        return result;
    }
}
