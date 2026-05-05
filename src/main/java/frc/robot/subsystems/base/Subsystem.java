package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.io.base.IO;
import frc.robot.io.base.State;
import frc.robot.robots.tusk.TuskState;
import frc.robot.util.Container;

public abstract class Subsystem<T extends State<T, X>, U extends Request<T, U, X>, V extends IO<T, V, X>, W extends Enum<W>, X extends Enum<X>> {
    protected final W m_name;

    protected Container<V> m_io;
    protected Alert m_missingIO = new Alert("", AlertType.kError);
    protected Requestor<T, U, W, X> m_requestor;
    protected Choreographer<T, U, V, W, X> m_choreographer;
    protected T m_state;
    protected U m_request;

    public Subsystem(W name) {
        m_name = name;
    }

    public void createIOChangeTriggers() {
        new Trigger(DriverStation::isDSAttached).onTrue(
            Commands
                .runOnce(() -> m_io.set(m_io.get().getProperIO()))
                .ignoringDisable(true)
        );

        new Trigger(() -> m_state.CurrentIdentity != m_state.ProperIdentity).onTrue(
            Commands
                .runOnce(() -> m_io.set(m_io.get().getProperIO()))
                .ignoringDisable(true)
        );
    }

    public void updateMissingIO() {}

    public T getState() {
        return m_state;
    }

    public T update() {
        return m_state.update(m_io.get().updateState());
    }

    public U requestTusk(TuskState fullState) {
        return m_request.update(m_requestor.requestTusk(fullState));
    }

    public void runTusk(TuskState fullState) {
        m_choreographer.choreographTusk(fullState, m_request);
    }
}
