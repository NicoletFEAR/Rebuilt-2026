package frc.robot.subsystems.base.tusk;

import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.State;

public abstract class TuskRequestor<T extends State<T, U, V>, U extends Enum<U>, V extends Enum<V>> {
    protected final U m_name;

    public TuskRequestor(U name) {
        m_name = name;
    }

    public abstract T request(TuskState fullState);
}
