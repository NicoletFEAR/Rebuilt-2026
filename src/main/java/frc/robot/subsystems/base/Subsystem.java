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

public abstract class Subsystem<T extends State<T, Y>, U extends State<U, Y>, V extends Request<U, V, Y>, W extends IO<T, W, Y>, X extends Enum<X>, Y extends Enum<Y>> {
    protected final X name;

    protected Container<W> io;
    protected Alert missingIO = new Alert("", AlertType.kError);
    protected Estimator<T, U, W, Y> estimator;
    protected Requestor<U, V, X, Y> requestor;
    protected Choreographer<T, U, V, W, X, Y> choreographer;
    protected U state;
    protected V request;

    public Subsystem(X name) {
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

    public U getState() {
        return state;
    }

    public U update() {
        return state.update(estimator.estimate());
    }

    public V requestTusk(TuskState fullState) {
        return request.update(requestor.request(fullState));
    }

    public void runTusk(TuskState fullState) {
        choreographer.choreograph(fullState, request);
    }
}
