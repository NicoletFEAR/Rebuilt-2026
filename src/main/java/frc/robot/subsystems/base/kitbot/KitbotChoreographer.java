package frc.robot.subsystems.base.kitbot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.subsystems.base.State;

public abstract class KitbotChoreographer<T extends State<T>> {
    public abstract Command choreograph(KitbotState fullState, T requestedState);
}
