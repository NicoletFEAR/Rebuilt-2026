package frc.robot.subsystems.base.kitbot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.subsystems.base.Inputs;
import frc.robot.subsystems.base.State;

public abstract class KitbotSubsystem<T extends State<T, V, W>, U extends Inputs<T, U, V, W>, V extends Enum<V>, W extends Enum<W>> {
    protected final V m_name;

    protected U m_inputs;
    protected Alert m_missingInputs = new Alert("", AlertType.kError);
    protected KitbotRequestor<T, V, W> m_requestor;
    protected KitbotChoreographer<T, U, V, W> m_choreographer;
    protected T m_state;
    protected T m_requestedState;

    public KitbotSubsystem(V name) {
        m_name = name;
        createInputsChangeTriggers();
    }

    public void createInputsChangeTriggers() {
        new Trigger(DriverStation::isDSAttached).onTrue(
            Commands
                .runOnce(() -> m_inputs = m_inputs.getInputs())
                .ignoringDisable(true)
        );

        new Trigger(() -> m_state.CurrentIdentity != m_state.ProperIdentity).onTrue(
            Commands
                .runOnce(() -> m_inputs = m_inputs.getInputs())
                .ignoringDisable(true)
        );
    }

    public abstract void updateMissingInputs();

    public T getState() {
        return m_state;
    }

    public T update() {
        return m_state.update(m_inputs.updateState());
    }

    public T request(KitbotState fullState) {
        return m_requestedState.update(m_requestor.request(fullState));
    }

    public void run(KitbotState fullState) {
        CommandScheduler.getInstance().schedule(m_choreographer.choreograph(fullState, m_requestedState));
    }
}
