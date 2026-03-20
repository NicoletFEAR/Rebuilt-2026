package frc.robot.subsystems.turn;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robots.hades.HadesState;
import frc.robot.subsystems.turn.io.TurnIO;

public class TurnChoreographer //extends UniversalChoreographer<TurnState, TurnIO, TurnName, TurnIdentity>
{
    public TurnChoreographer(TurnName name, TurnIO io) {
        //super(name, io);
    }

    //@Override
    public Command choreographHades(HadesState fullState, TurnState requestedState) {
        return Commands.none();
    }
}
