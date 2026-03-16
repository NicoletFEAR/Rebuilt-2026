package frc.robot.robots.kitbot;

import org.littletonrobotics.junction.Logger;

import frc.robot.robots.base.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public class Kitbot extends RobotContainer {
    private final Drive m_drive;

    private final KitbotStateAutoLogged m_state;
    private final KitbotStateAutoLogged m_requestedState;

    public Kitbot() {
        m_drive = new Drive();

        m_state = new KitbotStateAutoLogged();
        m_state.updateDriveState(m_drive.getState());
        m_requestedState = new KitbotState().update(m_state).toAutoLogged();
    }

    @Override
    public void periodic() {
        m_state.updateDriveState(m_drive.update());
        m_requestedState.updateDriveState(m_drive.requestKitbot(m_state));
        Logger.processInputs("State", m_state);
        Logger.processInputs("RequestedState", m_requestedState);
        m_drive.runKitbot(m_state);
    }
}
