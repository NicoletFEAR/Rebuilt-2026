package frc.robot.subsystems.controller.inputs;

import frc.robot.subsystems.controller.ControllerName;
import frc.robot.subsystems.controller.ControllerState;

public class ControllerInputsNone extends ControllerInputs {
    public ControllerInputsNone(ControllerName name, int port) {
        super(name, port);
        m_state = new ControllerState();
    }
}
