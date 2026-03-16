package frc.robot.subsystems.controller;

import frc.robot.subsystems.base.universal.UniversalSubsystem;
import frc.robot.subsystems.controller.inputs.ControllerInputs;
import frc.robot.subsystems.controller.inputs.ControllerInputsNone;

public class Controller extends UniversalSubsystem<ControllerState, ControllerInputs, ControllerName, ControllerIdentity> {
    public Controller(ControllerName name) {
        super(name);
        m_missingInputs.setText(String.format("%s Controller disconnected! (port %d)", m_name.toString(), m_name.getPort()));
        m_inputs = new ControllerInputsNone(m_name);
        m_requestor = new ControllerRequestor(m_name);
        m_choreographer = new ControllerChoreographer(m_name, m_inputs);
        m_state = new ControllerState();
        m_requestedState = new ControllerState();
    }

    @Override
    public void updateMissingInputs() {
        m_missingInputs.set(m_state.CurrentIdentity == ControllerIdentity.NONE);
    }
}
