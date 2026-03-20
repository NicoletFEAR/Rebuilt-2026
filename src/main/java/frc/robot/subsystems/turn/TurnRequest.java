package frc.robot.subsystems.turn;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.base.Request;
import frc.robot.util.constraint.AngleConstraintAutoLogged;
import frc.robot.util.constraint.AngularVelocityConstraintAutoLogged;
import frc.robot.util.constraint.VoltageConstraintAutoLogged;

@AutoLog
public class TurnRequest extends Request<TurnState, TurnRequest, TurnName, TurnIdentity> {
    public AngleConstraintAutoLogged Position = new AngleConstraintAutoLogged();
    public AngularVelocityConstraintAutoLogged Velocity = new AngularVelocityConstraintAutoLogged();
    public VoltageConstraintAutoLogged Voltage = new VoltageConstraintAutoLogged();

    @Override
    public TurnRequest update(TurnState newState) {
        Position.set(newState.Position);
        Velocity.set(newState.Velocity);
        Voltage.set(newState.Voltage);
        return this;
    }

    @Override
    public TurnRequest update(TurnRequest newRequest) {
        Position = newRequest.Position;
        Velocity = newRequest.Velocity;
        Voltage = newRequest.Voltage;
        return this;
    }

    public TurnRequestAutoLogged toAutoLogged() {
        TurnRequestAutoLogged result = new TurnRequestAutoLogged();
        result.update(this);
        return result;
    }
}
