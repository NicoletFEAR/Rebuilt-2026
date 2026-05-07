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
    protected final W name;

    protected Container<V> io;
    protected Alert missingIO = new Alert("", AlertType.kError);
    protected Requestor<T, U, W, X> requestor;
    protected Choreographer<T, U, V, W, X> choreographer;
    protected T state;
    protected U request;

    public Subsystem(W name) {
        this.name = name;
    }

    public void createIOChangeTriggers() {
        new Trigger(DriverStation::isDSAttached).onTrue(
            Commands
                .runOnce(() -> io.set(io.get().getProperIO()))
                .ignoringDisable(true)
        );

        new Trigger(() -> state.CurrentIdentity != state.ProperIdentity).onTrue(
            Commands
                .runOnce(() -> io.set(io.get().getProperIO()))
                .ignoringDisable(true)
        );
    }

    public void updateMissingIO() {}

    public T getState() {
        return state;
    }

    public T update() {
        return state.update(io.get().updateState());
    }

    public U requestTusk(TuskState fullState) {
        return request.update(requestor.requestTusk(fullState));
    }

    public void runTusk(TuskState fullState) {
        choreographer.choreographTusk(fullState, request);
    }
}
