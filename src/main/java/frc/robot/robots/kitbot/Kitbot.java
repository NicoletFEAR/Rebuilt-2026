package frc.robot.robots.kitbot;

import org.littletonrobotics.junction.Logger;

import frc.robot.robots.base.RobotContainer;
import frc.robot.subsystems.controller.Controller;
import frc.robot.subsystems.controller.ControllerName;

public class Kitbot extends RobotContainer {
    private final Controller m_controller;

    private final KitbotStateAutoLogged m_state = new KitbotStateAutoLogged();
    private final KitbotRequestAutoLogged m_request = new KitbotRequestAutoLogged();

    public Kitbot() {
        m_controller = new Controller(ControllerName.DRIVER);
    }

    @Override
    public void periodic() {
        m_state.updateController(m_controller.update());

        m_controller.updateMissingIO();

        m_request.updateController(m_controller.requestKitbot(m_state));

        Logger.processInputs("Kitbot/State", m_state);
        Logger.processInputs("Kitbot/Request", m_request);

        m_controller.runKitbot(m_state);
    }
}
