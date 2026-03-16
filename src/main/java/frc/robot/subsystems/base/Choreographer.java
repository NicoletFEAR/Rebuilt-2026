package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robots.hades.HadesState;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.robots.tusk.TuskState;

public abstract class Choreographer<T extends State<T>> {
    public abstract Command choreographHades(HadesState state, T requestedState);
    public abstract Command choreographKitbot(KitbotState state, T requestedState);
    public abstract Command choreographTusk(TuskState state, T requestedState);
}
