package frc.robot.robots.tusk;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.DeviceIds;
import frc.robot.robots.base.RobotContainer;
import frc.robot.subsystems.controller.Controller;
import frc.robot.subsystems.controller.ControllerName;
// import frc.robot.subsystems.drive.Drive;

public class Tusk extends RobotContainer {
    // private final Drive m_drive;
    private final Controller m_driverController;
    private final Controller m_operatorController;

    private final TuskStateAutoLogged m_state = new TuskStateAutoLogged();
    private final TuskStateAutoLogged m_requestedState = new TuskStateAutoLogged();

    public Tusk() {
        // m_drive = new Drive();
        m_driverController = new Controller(ControllerName.DRIVER, DeviceIds.kDriverController);
        m_operatorController = new Controller(ControllerName.OPERATOR, DeviceIds.kOperatorController);
    }

    @Override
    public void periodic() {
        // m_state.updateDriveState(m_drive.update());
        m_state.updateDriverControllerState(m_driverController.update());
        m_state.updateOperatorControllerState(m_operatorController.update());

        m_driverController.updateMissingInputs();
        m_operatorController.updateMissingInputs();

        // m_requestedState.updateDriveState(m_drive.requestTusk(m_state));
        m_requestedState.updateDriverControllerState(m_driverController.requestTusk(m_state));
        m_requestedState.updateOperatorControllerState(m_operatorController.requestTusk(m_state));

        Logger.processInputs("Tusk/State", m_state);
        Logger.processInputs("Tusk/RequestedState", m_requestedState);

        // m_drive.runTusk(m_state);
        m_driverController.runTusk(m_state);
        m_operatorController.runTusk(m_state);
    }
}
