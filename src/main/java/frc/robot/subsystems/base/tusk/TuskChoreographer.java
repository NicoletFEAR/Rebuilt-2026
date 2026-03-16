package frc.robot.subsystems.base.tusk;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.State;

public abstract class TuskChoreographer<T extends State<T>> {
    public abstract Command choreograph(TuskState fullState, T requestedState);
}
