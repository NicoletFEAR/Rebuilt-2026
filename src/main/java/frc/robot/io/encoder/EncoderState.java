package frc.robot.io.encoder;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.io.base.State;

@AutoLog
public class EncoderState extends State<EncoderState, EncoderIdentity> {
    public Angle Position = Radians.of(0.0);
    public AngularVelocity Velocity = RadiansPerSecond.of(0.0);

    @Override
    public EncoderState update(EncoderState newState) {
        CurrentIdentity = newState.CurrentIdentity;
        Position = newState.Position;
        ProperIdentity = newState.ProperIdentity;
        Velocity = newState.Velocity;
        return this;
    }

    public EncoderStateAutoLogged toAutoLogged() {
        EncoderStateAutoLogged result = new EncoderStateAutoLogged();
        result.update(this);
        return result;
    }
}
