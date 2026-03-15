package frc.robot.subsystems.base;

public abstract class State<T extends State<T>> {
    public abstract T update(T newState);
}
