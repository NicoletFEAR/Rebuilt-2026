package frc.robot.subsystems.base.kitbot;

import frc.robot.robots.kitbot.KitbotState;
import frc.robot.subsystems.base.State;

public abstract class KitbotRequestor<T extends State<T ,U, V>, U extends Enum<U>, V extends Enum<V>> {
    protected final U m_name;

    public KitbotRequestor(U name) {
        m_name = name;
    }

    public abstract T request(KitbotState fullState);
}
