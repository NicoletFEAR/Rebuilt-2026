package frc.robot.subsystems.controller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.base.universal.UniversalSubsystem;
import frc.robot.subsystems.controller.inputs.ControllerInputs;
import frc.robot.subsystems.controller.inputs.ControllerInputsNone;
import frc.robot.util.Container;

public class Controller extends UniversalSubsystem<ControllerState, ControllerInputs> {
    private ControllerName m_name;

    private int m_port;

    public Controller(ControllerName name, int port) {
        m_name = name;
        m_inputs = new Container<ControllerInputs>(new ControllerInputsNone(m_name, port));
        m_requestor = new ControllerRequestor(m_name);
        m_choreographer = new ControllerChoreographer(m_name, m_inputs);
        m_state = new ControllerState();
        m_requestedState = new ControllerState();
        createIdentityChangeTriggers();
    }

    public void createIdentityChangeTriggers() {
        new Trigger(DriverStation::isDSAttached)
            .onTrue(
                Commands
                    .runOnce(() -> m_inputs.set(m_state.ProperType.getInputs(m_name, m_port)))
                    .ignoringDisable(true)
            );

        new Trigger(() -> m_state.CurrentType != m_state.ProperType)
            .onTrue(
                Commands
                    .runOnce(() -> m_inputs.set(m_state.ProperType.getInputs(m_name, m_port)))
                    .ignoringDisable(true)
            );
    }
}
