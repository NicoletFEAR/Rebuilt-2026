package frc.robot.subsystems.base.kitbot;

import frc.robot.robots.kitbot.KitbotState;
import frc.robot.subsystems.base.State;

public abstract class KitbotRequestor<T extends State<T>> {
    public abstract T request(KitbotState fullState);
}
