package frc.robot.io.base;

public abstract class IO<T extends State<T, V>, U extends IO<T, U, V>, V extends Enum<V>> {
    protected T m_state;
    public abstract T updateState();
    public abstract U getProperIO();
}
