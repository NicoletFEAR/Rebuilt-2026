package frc.robot.subsystems.turn;

import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.base.Subsystem;
import frc.robot.subsystems.turn.io.TurnIO;
import frc.robot.subsystems.turn.io.TurnIONone;
import frc.robot.util.Container;

public class Turn extends Subsystem<TurnState, TurnRequest, TurnIO, TurnName, TurnIdentity> {
    private DriveConstants m_driveConstants;

    public Turn(TurnName name, DriveConstants driveConstants) {
        super(name);
        m_driveConstants = driveConstants;

        m_state = new TurnState();
        m_request = new TurnRequest();

        m_missingIO.setText(String.format(
            "%s Module Turn disconnected! (Motor at %s + Absolute encoder at %s)",
            m_name.toString(),
            m_name.getMotorId().toString(),
            m_name.getAbsoluteEncoderId().toString()
        ));

        m_io = new Container<TurnIO>(new TurnIONone(m_name, m_driveConstants));
        m_requestor = new TurnRequestor(m_name);
        m_choreographer = new TurnChoreographer(m_name, m_io);

        createIOChangeTriggers();
    }

    @Override
    public void updateMissingIO() {
        m_missingIO.set(m_state.CurrentIdentity == TurnIdentity.NONE);
    }
}
