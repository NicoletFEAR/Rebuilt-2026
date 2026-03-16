package frc.robot.subsystems.base.tusk;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.Inputs;
import frc.robot.subsystems.base.State;
import frc.robot.util.Container;

public abstract class TuskSubsystem<T extends State<T>, U extends Inputs<T>> {
    protected Container<U> m_inputs;
    protected Alert m_missingInputs = new Alert("", AlertType.kError);
    protected TuskRequestor<T> m_requestor;
    protected TuskChoreographer<T> m_choreographer;
    protected T m_state;
    protected T m_requestedState;

    public abstract void updateMissingInputs();

    public T getState() {
        return m_state;
    }

    public T update() {
        return m_state.update(m_inputs.get().updateState());
    }

    public T request(TuskState fullState) {
        return m_requestedState.update(m_requestor.request(fullState));
    }

    public void run(TuskState fullState) {
        CommandScheduler.getInstance().schedule(m_choreographer.choreograph(fullState, m_requestedState));
    }
}
