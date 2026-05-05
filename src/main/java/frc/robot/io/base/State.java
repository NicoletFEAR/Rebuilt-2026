package frc.robot.io.base;

public abstract class State<T extends State<T, U>, U extends Enum<U>> {
    public U CurrentIdentity;
    public U ProperIdentity;

    public abstract T update(T newState);
}
