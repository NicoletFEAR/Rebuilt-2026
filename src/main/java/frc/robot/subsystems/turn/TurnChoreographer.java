package frc.robot.subsystems.turn;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robots.hades.HadesState;
import frc.robot.robots.kitbot.KitbotState;
import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.Choreographer;
import frc.robot.subsystems.turn.io.TurnIO;
import frc.robot.util.Container;
import frc.robot.util.constraint.ConstraintType;

public class TurnChoreographer extends Choreographer<TurnState, TurnRequest, TurnIO, TurnName, TurnIdentity> {
    public TurnChoreographer(TurnName name, Container<TurnIO> io) {
        super(name, io);
    }

    @Override
    public Command choreographHades(HadesState fullState, TurnRequest requestedState) {
        ConstraintType voltageType = requestedState.Voltage.getType();
        Voltage voltage = requestedState.Voltage.get();

        if (voltageType == ConstraintType.IGNORE) {
            return Commands.none();
        } else {
            return m_io.get().applyVoltage(voltage);
        }
    }

    @Override
    public Command choreographKitbot(KitbotState fullState, TurnRequest requestedState) {
        ConstraintType voltageType = requestedState.Voltage.getType();
        Voltage voltage = requestedState.Voltage.get();

        if (voltageType == ConstraintType.IGNORE) {
            return Commands.none();
        } else {
            return m_io.get().applyVoltage(voltage);
        }
    }

    @Override
    public Command choreographTusk(TuskState fullState, TurnRequest requestedState) {
        ConstraintType voltageType = requestedState.Voltage.getType();
        Voltage voltage = requestedState.Voltage.get();

        if (voltageType == ConstraintType.IGNORE) {
            return Commands.none();
        } else {
            return m_io.get().applyVoltage(voltage);
        }
    }
}
