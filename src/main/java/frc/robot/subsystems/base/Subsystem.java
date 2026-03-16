package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.robots.hades.HadesState;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.robots.tusk.TuskState;

public abstract class Subsystem<T extends State<T>> {
    private Inputs<T> m_inputs;
    private Requestor<T> m_requestor;
    private Choreographer<T> m_choreographer;
    private T m_state;
    private T m_requestedState;

    public T getState() {
        return m_state;
    }

    public T update() {
        return m_state.update(m_inputs.updateState());
    }

    public T requestHades(HadesState fullState) {
        return m_requestedState.update(m_requestor.requestHadesState(fullState));
    }

    public void runHades(HadesState fullState) {
        CommandScheduler.getInstance().schedule(m_choreographer.choreographHades(fullState, m_requestedState));
    }

    public T requestKitbot(KitbotState fullState) {
        return m_requestedState.update(m_requestor.requestKitbotState(fullState));
    }

    public void runKitbot(KitbotState fullState) {
        CommandScheduler.getInstance().schedule(m_choreographer.choreographKitbot(fullState, m_requestedState));
    }

    public T requestTusk(TuskState fullState) {
        return m_requestedState.update(m_requestor.requestTuskState(fullState));
    }

    public void runTusk(TuskState fullState) {
        CommandScheduler.getInstance().schedule(m_choreographer.choreographTusk(fullState, m_requestedState));
    }
}
