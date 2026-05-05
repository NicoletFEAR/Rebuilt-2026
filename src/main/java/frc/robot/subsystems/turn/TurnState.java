package frc.robot.subsystems.turn;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.io.base.State;

@AutoLog
public class TurnState extends State<TurnState, TurnName, TurnIdentity> {
    public Angle Position = Radians.of(0.0);
    public AngularVelocity Velocity = RadiansPerSecond.of(0.0);
    public Voltage Voltage = Volts.of(0.0);

    @Override
    public TurnState update(TurnState newState) {
        CurrentIdentity = newState.CurrentIdentity;
        Name = newState.Name;
        Position = newState.Position;
        ProperIdentity = newState.ProperIdentity;
        Velocity = newState.Velocity;
        Voltage = newState.Voltage;
        return this;
    }

    public TurnStateAutoLogged toAutoLogged() {
        TurnStateAutoLogged result = new TurnStateAutoLogged();
        result.update(this);
        return result;
    }
}
