package frc.robot.subsystems.turn;

import static edu.wpi.first.units.Units.Volts;

import frc.robot.robots.tusk.TuskState;
import frc.robot.subsystems.base.Requestor;
import frc.robot.util.constraint.ConstraintType;

public class TurnRequestor extends Requestor<TurnState, TurnRequest, TurnName, TurnIdentity> {
    public TurnRequestor(TurnName name) {
        super(name);
        m_request = new TurnRequest();
    }

    @Override
    public TurnRequest requestTusk(TuskState fullState) {
        m_request.update(switch (m_name) {
            case FRONT_LEFT -> fullState.FrontLeftTurn;
            case FRONT_RIGHT -> fullState.FrontRightTurn;
            case REAR_LEFT -> fullState.RearLeftTurn;
            case REAR_RIGHT -> fullState.RearRightTurn;
        });

        m_request.Voltage.setType(ConstraintType.MATCH);
        m_request.Voltage.set(Volts.of(fullState.DriverController.LeftY * 12.0));

        return m_request;
    }
}
