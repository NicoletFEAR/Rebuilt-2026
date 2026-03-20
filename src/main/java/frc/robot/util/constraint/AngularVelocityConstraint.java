package frc.robot.util.constraint;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

@AutoLog
public class AngularVelocityConstraint extends Constraint<AngularVelocity> {
    public AngularVelocityConstraint() {
        Value = RadiansPerSecond.of(0.0);
    }

    public AngularVelocityConstraintAutoLogged toAutoLogged() {
        AngularVelocityConstraintAutoLogged result = new AngularVelocityConstraintAutoLogged();
        result.update(this);
        return result;
    }
}
