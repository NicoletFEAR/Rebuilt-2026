package frc.robot.subsystems.base.universal;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.robots.hades.HadesState;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.Inputs;
import frc.robot.subsystems.base.State;
import frc.robot.util.Container;

public abstract class UniversalSubsystem<T extends State<T>, U extends Inputs<T>> {
    protected Container<U> m_inputs;
    protected Alert m_missingInputs = new Alert("", AlertType.kError);
    protected UniversalRequestor<T> m_requestor;
    protected UniversalChoreographer<T> m_choreographer;
    protected T m_state;
    protected T m_requestedState;

    public abstract void updateMissingInputs();

    public T getState() {
        return m_state;
    }

    public T update() {
        return m_state.update(m_inputs.get().updateState());
    }

    public T requestHades(HadesState fullState) {
        return m_requestedState.update(m_requestor.requestHades(fullState));
    }

    public void runHades(HadesState fullState) {
        CommandScheduler.getInstance().schedule(m_choreographer.choreographHades(fullState, m_requestedState));
    }

    public T requestKitbot(KitbotState fullState) {
        return m_requestedState.update(m_requestor.requestKitbot(fullState));
    }

    public void runKitbot(KitbotState fullState) {
        CommandScheduler.getInstance().schedule(m_choreographer.choreographKitbot(fullState, m_requestedState));
    }

    public T requestTusk(TuskState fullState) {
        return m_requestedState.update(m_requestor.requestTusk(fullState));
    }

    public void runTusk(TuskState fullState) {
        CommandScheduler.getInstance().schedule(m_choreographer.choreographTusk(fullState, m_requestedState));
    }
}
