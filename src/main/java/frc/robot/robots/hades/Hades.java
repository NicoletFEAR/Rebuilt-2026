package frc.robot.robots.hades;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.DeviceIds;
import frc.robot.robots.base.RobotContainer;
import frc.robot.subsystems.controller.Controller;
import frc.robot.subsystems.controller.ControllerName;
import frc.robot.subsystems.drive.Drive;

public class Hades extends RobotContainer {
    private final Controller m_controller;
    private final Drive m_drive;

    private final HadesStateAutoLogged m_state = new HadesStateAutoLogged();
    private final HadesStateAutoLogged m_requestedState = new HadesStateAutoLogged();

    public Hades() {
        m_controller = new Controller(ControllerName.DRIVER, DeviceIds.kDriverController);
        m_drive = new Drive();
    }

    @Override
    public void periodic() {
        m_state.updateControllerState(m_controller.update());
        m_state.updateDriveState(m_drive.update());

        m_controller.updateMissingInputs();
        m_drive.updateMissingInputs();

        m_requestedState.updateControllerState(m_controller.requestHades(m_state));
        m_requestedState.updateDriveState(m_drive.requestHades(m_state));

        Logger.processInputs("Hades/State", m_state);
        Logger.processInputs("Hades/RequestedState", m_requestedState);

        m_controller.runHades(m_state);
        m_drive.runHades(m_state);
    }
}
