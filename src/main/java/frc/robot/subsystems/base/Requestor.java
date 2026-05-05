package frc.robot.subsystems.base;

import frc.robot.io.base.State;
import frc.robot.robots.tusk.TuskState;

public abstract class Requestor<T extends State<T, W>, U extends Request<T, U, W>, V extends Enum<V>, W extends Enum<W>> {
    protected final V m_name;

    protected U m_request;

    public Requestor(V name) {
        m_name = name;
    }

    public abstract U requestTusk(TuskState fullState);
}
