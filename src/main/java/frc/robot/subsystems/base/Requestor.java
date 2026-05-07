package frc.robot.subsystems.base;

import frc.robot.io.base.State;
import frc.robot.robots.tusk.TuskState;

public abstract class Requestor<T extends State<T, W>, U extends Request<T, U, W>, V extends Enum<V>, W extends Enum<W>> {
    protected final V name;

    protected U request;

    public Requestor(V name) {
        this.name = name;
    }

    public abstract U request(TuskState fullState);
}
