package frc.robot.util;

public class Container<T> {
    private T item;

    public Container(T item) {
        this.item = item;
    }

    public T get() {
        return item;
    }

    public void set(T newItem) {
        item = newItem;
    }
}
