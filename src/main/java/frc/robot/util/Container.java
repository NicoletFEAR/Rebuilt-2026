package frc.robot.util;

public class Container<T> {
    private T m_item;

    public Container(T item) {
        m_item = item;
    }

    public T get() {
        return m_item;
    }

    public void set(T newItem) {
        m_item = newItem;
    }
}
