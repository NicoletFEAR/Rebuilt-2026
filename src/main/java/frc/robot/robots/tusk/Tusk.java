package frc.robot.robots.tusk;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.drive.Drive;

public class Tusk {
    private final Drive m_drive;

    private final TuskStateAutoLogged m_state;
    private final TuskStateAutoLogged m_requestedState;

    public Tusk() {
        m_drive = new Drive();

        m_state = new TuskStateAutoLogged();
        m_state.updateDriveState(m_drive.getState());
        m_requestedState = new TuskStateAutoLogged().update(m_state).toAutoLogged();
    }

    public void periodic() {
        m_state.updateDriveState(m_drive.update());
        m_requestedState.updateDriveState(m_drive.request(m_state));
        Logger.processInputs("State", m_state);
        Logger.processInputs("RequestedState", m_requestedState);
        m_drive.run(m_state);
    }
}
