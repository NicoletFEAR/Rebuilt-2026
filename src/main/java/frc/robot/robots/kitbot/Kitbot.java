package frc.robot.robots.kitbot;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.DeviceIds;
import frc.robot.robots.base.RobotContainer;
import frc.robot.subsystems.controller.Controller;
import frc.robot.subsystems.controller.ControllerName;
import frc.robot.subsystems.drive.Drive;

public class Kitbot extends RobotContainer {
    private final Controller m_controller;
    private final Drive m_drive;

    private final KitbotStateAutoLogged m_state = new KitbotStateAutoLogged();
    private final KitbotStateAutoLogged m_requestedState = new KitbotStateAutoLogged();

    public Kitbot() {
        m_controller = new Controller(ControllerName.DRIVER, DeviceIds.kDriverController);
        m_drive = new Drive();
    }

    @Override
    public void periodic() {
        m_state.updateControllerState(m_controller.update());
        m_state.updateDriveState(m_drive.update());
        
        m_controller.updateMissingInputs();
        m_drive.updateMissingInputs();

        m_requestedState.updateControllerState(m_controller.requestKitbot(m_state));
        m_requestedState.updateDriveState(m_drive.requestKitbot(m_state));

        Logger.processInputs("Kitbot/State", m_state);
        Logger.processInputs("Kitbot/RequestedState", m_requestedState);

        m_controller.runKitbot(m_state);
        m_drive.runKitbot(m_state);
    }
}
