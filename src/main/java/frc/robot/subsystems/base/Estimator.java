package frc.robot.subsystems.base;

import frc.robot.io.base.IO;
import frc.robot.io.base.State;
import frc.robot.util.Container;

public abstract class Estimator<T extends State<T, W>, U extends State<U, W>, V extends IO<T, V, W>, W extends Enum<W>> {
    protected final Container<V> io;

    protected T state;
    protected U estimate;

    public Estimator(Container<V> io) {
        this.io = io;
    }

    public abstract U estimate();
}
