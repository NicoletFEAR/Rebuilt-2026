package frc.robot.subsystems.turn;

import edu.wpi.first.units.measure.Voltage;
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
    public void choreographTusk(TuskState fullState, TurnRequest requestedState) {
        ConstraintType voltageType = requestedState.Voltage.getType();
        Voltage voltage = requestedState.Voltage.get();

        if (voltageType == ConstraintType.MATCH) {
            m_io.get().applyVoltage(voltage);
        }
    }
}
