package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

    public T request(TuskState fullState) {
        return m_requestedState.update(m_requestor.requestState(fullState));
    }

    public void run(TuskState fullState) {
        CommandScheduler.getInstance().schedule(m_choreographer.choreograph(fullState, m_requestedState));
    }
}
