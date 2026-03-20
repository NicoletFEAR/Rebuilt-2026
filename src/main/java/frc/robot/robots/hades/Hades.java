package frc.robot.robots.hades;

import org.littletonrobotics.junction.Logger;

import frc.robot.robots.base.RobotContainer;
import frc.robot.subsystems.controller.Controller;
import frc.robot.subsystems.controller.ControllerName;

public class Hades extends RobotContainer {
    private final Controller m_controller;

    private final HadesStateAutoLogged m_state = new HadesStateAutoLogged();
    private final HadesRequestAutoLogged m_request = new HadesRequestAutoLogged();

    public Hades() {
        m_controller = new Controller(ControllerName.DRIVER);
    }

    @Override
    public void periodic() {
        m_state.updateController(m_controller.update());

        m_controller.updateMissingIO();

        m_request.updateController(m_controller.requestHades(m_state));

        Logger.processInputs("Hades/State", m_state);
        Logger.processInputs("Hades/Request", m_request);

        m_controller.runHades(m_state);
    }
}
