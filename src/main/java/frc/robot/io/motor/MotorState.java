package frc.robot.io.motor;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.io.base.State;

@AutoLog
public class MotorState extends State<MotorState, MotorIdentity> {
    public Angle Position = Radians.of(0.0);
    public AngularVelocity Velocity = RadiansPerSecond.of(0.0);
    public Voltage Voltage = Volts.of(0.0);

    @Override
    public MotorState update(MotorState newState) {
        CurrentIdentity = newState.CurrentIdentity;
        Position = newState.Position;
        ProperIdentity = newState.ProperIdentity;
        Velocity = newState.Velocity;
        Voltage = newState.Voltage;
        return this;
    }

    public MotorStateAutoLogged toAutoLogged() {
        MotorStateAutoLogged result = new MotorStateAutoLogged();
        result.update(this);
        return result;
    }
}
