package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robots.hades.HadesState;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.robots.tusk.TuskState;
import frc.robot.util.Container;

public abstract class Choreographer<T extends State<T, W, X>, U extends Request<T, U, W, X>, V extends IO<T, V, W, X>, W extends Enum<W>, X extends Enum<X>> {
    protected final W m_name;

    protected final Container<V> m_io;

    public Choreographer(W name, Container<V> io) {
        m_name = name;
        m_io = io;
    }

    public Command choreographHades(HadesState fullState, U requestedState) {
        return Commands.none();
    }

    public Command choreographKitbot(KitbotState fullState, U requestedState) {
        return Commands.none();
    }

    public Command choreographTusk(TuskState fullState, U requestedState) {
        return Commands.none();
    }
}
