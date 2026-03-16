package frc.robot.robots.hades;

import org.littletonrobotics.junction.Logger;

import frc.robot.robots.base.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public class Hades extends RobotContainer {
    private final Drive m_drive;

    private final HadesStateAutoLogged m_state;
    private final HadesStateAutoLogged m_requestedState;

    public Hades() {
        m_drive = new Drive();

        m_state = new HadesStateAutoLogged();
        m_state.updateDriveState(m_drive.getState());
        m_requestedState = new HadesState().update(m_state).toAutoLogged();
    }

    @Override
    public void periodic() {
        m_state.updateDriveState(m_drive.update());
        m_requestedState.updateDriveState(m_drive.requestHades(m_state));
        Logger.processInputs("State", m_state);
        Logger.processInputs("RequestedState", m_requestedState);
        m_drive.runHades(m_state);
    }
}
