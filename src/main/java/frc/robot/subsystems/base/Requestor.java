package frc.robot.subsystems.base;

import frc.robot.robots.tusk.TuskState;

public abstract class Requestor<T extends State<T>> {
    public abstract T requestState(TuskState fullState);
}
