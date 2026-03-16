package frc.robot.robots.kitbot;

import org.littletonrobotics.junction.Logger;

import frc.robot.robots.base.RobotContainer;
import frc.robot.subsystems.controller.Controller;
import frc.robot.subsystems.controller.ControllerName;

public class Kitbot extends RobotContainer {
    private final Controller m_controller;

    private final KitbotStateAutoLogged m_state = new KitbotStateAutoLogged();
    private final KitbotStateAutoLogged m_requestedState = new KitbotStateAutoLogged();

    public Kitbot() {
        m_controller = new Controller(ControllerName.DRIVER);
    }

    @Override
    public void periodic() {
        m_state.updateControllerState(m_controller.update());

        m_controller.updateMissingInputs();

        m_requestedState.updateControllerState(m_controller.requestKitbot(m_state));

        Logger.processInputs("Kitbot/State", m_state);
        Logger.processInputs("Kitbot/RequestedState", m_requestedState);

        m_controller.runKitbot(m_state);
    }
}
