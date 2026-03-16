package frc.robot.subsystems.base.kitbot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.subsystems.base.Inputs;
import frc.robot.subsystems.base.State;

public abstract class KitbotChoreographer<T extends State<T, V, W>, U extends Inputs<T, U, V, W>, V extends Enum<V>, W extends Enum<W>> {
    protected V m_name;

    protected U m_inputs;

    public KitbotChoreographer(V name, U inputs) {
        m_name = name;
        m_inputs = inputs;
    }

    public abstract Command choreograph(KitbotState fullState, T requestedState);
}
