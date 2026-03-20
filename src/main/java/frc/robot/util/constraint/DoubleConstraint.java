package frc.robot.util.constraint;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DoubleConstraint {
    protected ConstraintType Type;
    protected double Value;

    public double get() {
        return Value;
    }

    public ConstraintType getType() {
        return Type;
    }

    public void set(double newValue) {
        Value = newValue;
    }

    public void update(DoubleConstraint newConstraint) {
        Type = newConstraint.Type;
        Value = newConstraint.Value;
    }

    public DoubleConstraintAutoLogged toAutoLogged() {
        DoubleConstraintAutoLogged result = new DoubleConstraintAutoLogged();
        result.update(this);
        return result;
    }
}
