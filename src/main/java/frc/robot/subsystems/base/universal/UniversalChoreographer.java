package frc.robot.subsystems.base.universal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robots.hades.HadesState;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.State;

public abstract class UniversalChoreographer<T extends State<T>> {
    public abstract Command choreographHades(HadesState fullState, T requestedState);
    public abstract Command choreographKitbot(KitbotState fullState, T requestedState);
    public abstract Command choreographTusk(TuskState fullState, T requestedState);
}
