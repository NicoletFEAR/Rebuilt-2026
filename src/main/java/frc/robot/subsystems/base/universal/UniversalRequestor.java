package frc.robot.subsystems.base.universal;

import frc.robot.robots.hades.HadesState;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.State;

public abstract class UniversalRequestor<T extends State<T>> {
    public abstract T requestHades(HadesState fullState);
    public abstract T requestKitbot(KitbotState fullState);
    public abstract T requestTusk(TuskState fullState);
}
