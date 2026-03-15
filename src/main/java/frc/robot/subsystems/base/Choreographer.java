package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robots.tusk.TuskState;

public abstract class Choreographer<T extends State<T>> {
    public abstract Command choreograph(TuskState state, T requestedState);
}
