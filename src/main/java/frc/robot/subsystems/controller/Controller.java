package frc.robot.subsystems.controller;

import frc.robot.subsystems.base.Subsystem;
import frc.robot.subsystems.controller.io.ControllerIO;
import frc.robot.subsystems.controller.io.ControllerIONone;
import frc.robot.util.Container;

public class Controller extends Subsystem<ControllerState, ControllerRequest, ControllerIO, ControllerName, ControllerIdentity> {
    public Controller(ControllerName name) {
        super(name);

        m_state = new ControllerState();
        m_request = new ControllerRequest();

        m_missingIO.setText(String.format("%s Controller disconnected! (port %d)", m_name.toString(), m_name.getPort()));
        m_io = new Container<ControllerIO>(new ControllerIONone(m_name));
        m_requestor = new ControllerRequestor(m_name);
        m_choreographer = new ControllerChoreographer(m_name, m_io);

        createIOChangeTriggers();
    }

    @Override
    public void updateMissingIO() {
        m_missingIO.set(m_state.CurrentIdentity == ControllerIdentity.NONE);
    }
}
