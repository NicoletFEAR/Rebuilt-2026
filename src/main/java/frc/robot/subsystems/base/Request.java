package frc.robot.subsystems.base;

import frc.robot.io.base.State;

public abstract class Request<T extends State<T, V>, U extends Request<T, U, V>, V extends Enum<V>> {
    public abstract U update(T newState);
    public abstract U update(U newRequest);
}
