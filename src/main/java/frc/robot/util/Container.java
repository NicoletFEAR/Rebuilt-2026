package frc.robot.util;

public class Container<T> {
    private T m_value;

    public Container(T value) {
        m_value = value;
    }

    public T get() {
        return m_value;
    }

    public void set(T value) {
        m_value = value;
    }
}
