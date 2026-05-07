package frc.robot.subsystems.base;

import frc.robot.io.base.IO;
import frc.robot.io.base.State;
import frc.robot.robots.tusk.TuskState;
import frc.robot.util.Container;

public abstract class Choreographer<T extends State<T, Y>, U extends State<U, Y>, V extends Request<U, V, Y>, W extends IO<T, W, Y>, X extends Enum<X>, Y extends Enum<Y>> {
    protected final X name;

    protected final Container<W> io;

    public Choreographer(X name, Container<W> io) {
        this.name = name;
        this.io = io;
    }

    public void choreograph(TuskState fullState, V requestedState) {}
}
