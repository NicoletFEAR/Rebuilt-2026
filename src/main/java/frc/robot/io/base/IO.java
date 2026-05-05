package frc.robot.io.base;

public abstract class IO<T extends State<T, V, W>, U extends IO<T, U, V, W>, V extends Enum<V>, W extends Enum<W>> {
    protected V m_name;

    protected T m_state;

    public IO(V name) {
        m_name = name;
    }

    public abstract T updateState();

    public abstract U getProperIO();
}
