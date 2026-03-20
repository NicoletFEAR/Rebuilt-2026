package frc.robot.subsystems.base.kitbot;

import frc.robot.robots.kitbot.KitbotState;
import frc.robot.subsystems.base.Request;
import frc.robot.subsystems.base.State;

public abstract class KitbotRequestor<T extends State<T, V, W>, U extends Request<T, U, V, W>, V extends Enum<V>, W extends Enum<W>> {
    protected final V m_name;

    protected U m_request;

    public KitbotRequestor(V name) {
        m_name = name;
    }

    public abstract U request(KitbotState fullState);
}
