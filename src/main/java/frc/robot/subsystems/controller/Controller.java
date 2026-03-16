package frc.robot.subsystems.controller;

import frc.robot.subsystems.base.universal.UniversalSubsystem;

public class Controller extends UniversalSubsystem<ControllerState> {
    public Controller(int port, ControllerName name) {
        m_choreographer = new ControllerChoreographer(port, name);
        m_requestor = new ControllerRequestor(name);
        m_inputs = new ControllerInputs(port, name);
        m_state = new ControllerState();
        m_requestedState = new ControllerState();
    }
}
