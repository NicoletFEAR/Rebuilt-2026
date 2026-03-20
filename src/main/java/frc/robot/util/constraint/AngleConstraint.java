package frc.robot.util.constraint;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

@AutoLog
public class AngleConstraint extends Constraint<Angle> {
    public AngleConstraint() {
        Value = Radians.of(0.0);
    }

    public AngleConstraintAutoLogged toAutoLogged() {
        AngleConstraintAutoLogged result = new AngleConstraintAutoLogged();
        result.update(this);
        return result;
    }
}
