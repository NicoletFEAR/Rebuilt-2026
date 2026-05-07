package frc.robot.subsystems.base;

import frc.robot.io.base.IO;
import frc.robot.io.base.State;
import frc.robot.robots.tusk.TuskState;
import frc.robot.util.Container;

public abstract class Choreographer<T extends State<T, X>, U extends Request<T, U, X>, V extends IO<T, V, X>, W extends Enum<W>, X extends Enum<X>> {
    protected final W name;

    protected final Container<V> io;

    public Choreographer(W name, Container<V> io) {
        this.name = name;
        this.io = io;
    }

    public void choreographTusk(TuskState fullState, U requestedState) {}
}
