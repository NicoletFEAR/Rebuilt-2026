package frc.robot.subsystems.base;

public abstract class Inputs<T extends State<T, U, V>, U extends Enum<U>, V extends Enum<V>> {
    protected U m_name;

    protected T m_state;

    public Inputs(U name) {
        m_name = name;
    }

    public abstract T updateState();
}
