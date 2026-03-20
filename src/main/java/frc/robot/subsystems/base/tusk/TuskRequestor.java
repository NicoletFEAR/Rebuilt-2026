package frc.robot.subsystems.base.tusk;

import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.Request;
import frc.robot.subsystems.base.State;

public abstract class TuskRequestor<T extends State<T, V, W>, U extends Request<T, U, V, W>, V extends Enum<V>, W extends Enum<W>> {
    protected final V m_name;

    protected U m_request;

    public TuskRequestor(V name) {
        m_name = name;
    }

    public abstract U request(TuskState fullState);
}
