package frc.robot.robots.tusk;

import org.littletonrobotics.junction.Logger;

import frc.robot.robots.base.RobotContainer;
import frc.robot.subsystems.controller.Controller;
import frc.robot.subsystems.controller.ControllerName;

public class Tusk extends RobotContainer {
    private final Controller m_driverController;
    private final Controller m_operatorController;

    private final TuskStateAutoLogged m_state = new TuskStateAutoLogged();
    private final TuskRequestAutoLogged m_request = new TuskRequestAutoLogged();

    public Tusk() {
        m_driverController = new Controller(ControllerName.DRIVER);
        m_operatorController = new Controller(ControllerName.OPERATOR);
    }

    @Override
    public void periodic() {
        m_state.updateDriverController(m_driverController.update());
        m_state.updateOperatorController(m_operatorController.update());

        m_driverController.updateMissingIO();
        m_operatorController.updateMissingIO();

        m_request.updateDriverController(m_driverController.requestTusk(m_state));
        m_request.updateOperatorController(m_operatorController.requestTusk(m_state));

        Logger.processInputs("Tusk/State", m_state);
        Logger.processInputs("Tusk/Request", m_request);

        m_driverController.runTusk(m_state);
        m_operatorController.runTusk(m_state);
    }
}
