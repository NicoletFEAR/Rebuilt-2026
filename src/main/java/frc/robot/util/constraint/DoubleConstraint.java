package frc.robot.util.constraint;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DoubleConstraint extends Constraint<Double> {
    public DoubleConstraint() {
        Value = 0.0;
    }

    public DoubleConstraintAutoLogged toAutoLogged() {
        DoubleConstraintAutoLogged result = new DoubleConstraintAutoLogged();
        result.update(this);
        return result;
    }
}
