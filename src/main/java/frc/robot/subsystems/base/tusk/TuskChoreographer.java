package frc.robot.subsystems.base.tusk;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.IO;
import frc.robot.subsystems.base.Request;
import frc.robot.subsystems.base.State;

public abstract class TuskChoreographer<T extends State<T, W, X>, U extends Request<T, U, W, X>, V extends IO<T, V, W, X>, W extends Enum<W>, X extends Enum<X>> {
    protected final W m_name;

    protected final V m_io;

    public TuskChoreographer(W name, V io) {
        m_name = name;
        m_io = io;
    }

    public abstract Command choreograph(TuskState fullState, U requestedState);
}
