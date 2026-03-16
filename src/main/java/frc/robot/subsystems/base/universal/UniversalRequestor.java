package frc.robot.subsystems.base.universal;

import frc.robot.robots.hades.HadesState;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.State;

public abstract class UniversalRequestor<T extends State<T, U, V>, U extends Enum<U>, V extends Enum<V>> {
    protected final U m_name;

    public UniversalRequestor(U name) {
        m_name = name;
    }

    public abstract T requestHades(HadesState fullState);
    public abstract T requestKitbot(KitbotState fullState);
    public abstract T requestTusk(TuskState fullState);
}
