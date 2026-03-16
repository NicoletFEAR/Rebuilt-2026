package frc.robot.subsystems.controller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.base.universal.UniversalSubsystem;
import frc.robot.subsystems.controller.inputs.ControllerInputs;
import frc.robot.subsystems.controller.inputs.ControllerInputsNone;

public class Controller extends UniversalSubsystem<ControllerState, ControllerInputs, ControllerName, ControllerIdentity> {
    private final ControllerName m_name;

    private final int m_port;

    public Controller(ControllerName name, int port) {
        m_name = name;
        m_port = port;
        m_missingInputs.setText(String.format("%s Controller not detected! (port %d)", m_name.toString(), m_port));
        m_inputs = new ControllerInputsNone(m_name, m_port);
        m_requestor = new ControllerRequestor(m_name);
        m_choreographer = new ControllerChoreographer(m_name, m_inputs);
        m_state = new ControllerState();
        m_requestedState = new ControllerState();
        createInputsChangeTriggers();
    }

    public void createInputsChangeTriggers() {
        new Trigger(DriverStation::isDSAttached)
            .onTrue(
                Commands
                    .runOnce(() -> m_inputs = m_state.ProperIdentity.getInputs(m_name, m_port))
                    .ignoringDisable(true)
            );

        new Trigger(() -> m_state.CurrentIdentity != m_state.ProperIdentity)
            .onTrue(
                Commands
                    .runOnce(() -> m_inputs = m_state.ProperIdentity.getInputs(m_name, m_port))
                    .ignoringDisable(true)
            );
    }

    @Override
    public void updateMissingInputs() {
        m_missingInputs.set(m_state.CurrentIdentity == ControllerIdentity.NONE);
    }
}
