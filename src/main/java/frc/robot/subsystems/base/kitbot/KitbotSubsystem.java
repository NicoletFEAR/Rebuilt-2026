package frc.robot.subsystems.base.kitbot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.subsystems.base.IO;
import frc.robot.subsystems.base.Request;
import frc.robot.subsystems.base.State;

public abstract class KitbotSubsystem<T extends State<T, W, X>, U extends Request<T, U, W, X>, V extends IO<T, V, W, X>, W extends Enum<W>, X extends Enum<X>> {
    protected final W m_name;

    protected V m_io;
    protected Alert m_missingIO = new Alert("", AlertType.kError);
    protected KitbotRequestor<T, U, W, X> m_requestor;
    protected KitbotChoreographer<T, U, V, W, X> m_choreographer;
    protected T m_state;
    protected U m_request;

    public KitbotSubsystem(W name) {
        m_name = name;
        createIOChangeTriggers();
    }

    public void createIOChangeTriggers() {
        new Trigger(DriverStation::isDSAttached).onTrue(
            Commands
                .runOnce(() -> m_io = m_io.getProperIO())
                .ignoringDisable(true)
        );

        new Trigger(() -> m_state.CurrentIdentity != m_state.ProperIdentity).onTrue(
            Commands
                .runOnce(() -> m_io = m_io.getProperIO())
                .ignoringDisable(true)
        );
    }

    public abstract void updateMissingIO();

    public T getState() {
        return m_state;
    }

    public T update() {
        return m_state.update(m_io.updateState());
    }

    public U request(KitbotState fullState) {
        return m_request.update(m_requestor.request(fullState));
    }

    public void run(KitbotState fullState) {
        CommandScheduler.getInstance().schedule(m_choreographer.choreograph(fullState, m_request));
    }
}
