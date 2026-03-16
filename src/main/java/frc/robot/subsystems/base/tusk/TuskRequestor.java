package frc.robot.subsystems.base.tusk;

import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.State;

public abstract class TuskRequestor<T extends State<T>> {
    public abstract T request(TuskState fullState);
}
