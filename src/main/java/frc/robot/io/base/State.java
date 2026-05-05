package frc.robot.io.base;

public abstract class State<T extends State<T, U, V>, U extends Enum<U>, V extends Enum<V>> {
    public V CurrentIdentity;
    public U Name;
    public V ProperIdentity;

    public abstract T update(T newState);
}
