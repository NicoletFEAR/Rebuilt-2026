package frc.robot.subsystems.base;

public abstract class Inputs<T extends State<T, V, W>, U extends Inputs<T, U, V, W>, V extends Enum<V>, W extends Enum<W>> {
    protected V m_name;

    protected T m_state;

    public Inputs(V name) {
        m_name = name;
    }

    public abstract T updateState();

    public abstract U getInputs(W identity);
}
