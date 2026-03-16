package frc.robot.subsystems.base.tusk;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.Inputs;
import frc.robot.subsystems.base.State;

public abstract class TuskChoreographer<T extends State<T, V, W>, U extends Inputs<T, U, V, W>, V extends Enum<V>, W extends Enum<W>> {
    protected final V m_name;

    protected final U m_inputs;

    public TuskChoreographer(V name, U inputs) {
        m_name = name;
        m_inputs = inputs;
    }

    public abstract Command choreograph(TuskState fullState, T requestedState);
}
