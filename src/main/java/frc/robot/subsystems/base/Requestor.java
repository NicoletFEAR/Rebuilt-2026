package frc.robot.subsystems.base;

import frc.robot.robots.hades.HadesState;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.robots.tusk.TuskState;

public abstract class Requestor<T extends State<T>> {
    public abstract T requestHadesState(HadesState fullState);
    public abstract T requestKitbotState(KitbotState kitbotState);
    public abstract T requestTuskState(TuskState fullState);
}
