package frc.robot.robots.hades;

import org.littletonrobotics.junction.Logger;

import frc.robot.robots.base.RobotContainer;
import frc.robot.subsystems.controller.Controller;
import frc.robot.subsystems.controller.ControllerName;

public class Hades extends RobotContainer {
    private final Controller m_controller;

    private final HadesStateAutoLogged m_state = new HadesStateAutoLogged();
    private final HadesStateAutoLogged m_requestedState = new HadesStateAutoLogged();

    public Hades() {
        m_controller = new Controller(ControllerName.DRIVER);
    }

    @Override
    public void periodic() {
        m_state.updateControllerState(m_controller.update());

        m_controller.updateMissingInputs();

        m_requestedState.updateControllerState(m_controller.requestHades(m_state));

        Logger.processInputs("Hades/State", m_state);
        Logger.processInputs("Hades/RequestedState", m_requestedState);

        m_controller.runHades(m_state);
    }
}
