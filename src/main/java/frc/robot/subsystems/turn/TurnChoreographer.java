package frc.robot.subsystems.turn;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robots.hades.HadesState;
import frc.robot.subsystems.turn.inputs.TurnInputs;

public class TurnChoreographer //extends UniversalChoreographer<TurnState, TurnInputs, TurnName, TurnIdentity>
{
    public TurnChoreographer(TurnName name, TurnInputs inputs) {
        //super(name, inputs);
    }

    //@Override
    public Command choreographHades(HadesState fullState, TurnState requestedState) {
        return Commands.none();
    }
}
