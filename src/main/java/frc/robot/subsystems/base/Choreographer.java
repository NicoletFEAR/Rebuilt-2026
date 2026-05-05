package frc.robot.subsystems.base;

import frc.robot.io.base.IO;
import frc.robot.io.base.State;
import frc.robot.robots.tusk.TuskState;
import frc.robot.util.Container;

public abstract class Choreographer<T extends State<T, W, X>, U extends Request<T, U, W, X>, V extends IO<T, V, W, X>, W extends Enum<W>, X extends Enum<X>> {
    protected final W m_name;

    protected final Container<V> m_io;

    public Choreographer(W name, Container<V> io) {
        m_name = name;
        m_io = io;
    }

    public void choreographTusk(TuskState fullState, U requestedState) {}
}
