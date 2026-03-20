package frc.robot.subsystems.controller;

import frc.robot.subsystems.base.universal.UniversalSubsystem;
import frc.robot.subsystems.controller.io.ControllerIO;
import frc.robot.subsystems.controller.io.ControllerIONone;

public class Controller extends UniversalSubsystem<ControllerState, ControllerRequest, ControllerIO, ControllerName, ControllerIdentity> {
    public Controller(ControllerName name) {
        super(name);
        m_missingIO.setText(String.format("%s Controller disconnected! (port %d)", m_name.toString(), m_name.getPort()));
        m_io = new ControllerIONone(m_name);
        m_requestor = new ControllerRequestor(m_name);
        m_choreographer = new ControllerChoreographer(m_name, m_io);
        m_state = new ControllerState();
        m_request = new ControllerRequest();
    }

    @Override
    public void updateMissingIO() {
        m_missingIO.set(m_state.CurrentIdentity == ControllerIdentity.NONE);
    }
}
