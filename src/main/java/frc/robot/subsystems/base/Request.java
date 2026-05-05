package frc.robot.subsystems.base;

import frc.robot.io.base.State;

public abstract class Request<T extends State<T, V, W>, U extends Request<T, U, V, W>, V extends Enum<V>, W extends Enum<W>> {
    public abstract U update(T newState);
    public abstract U update(U newRequest);
}
