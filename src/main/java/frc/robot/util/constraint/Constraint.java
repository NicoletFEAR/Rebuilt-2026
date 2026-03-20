package frc.robot.util.constraint;

public class Constraint<T> {
    protected ConstraintType Type;
    protected T Value;

    public Constraint() {
        Type = ConstraintType.IGNORE;
    }

    public T get() {
        return Value;
    }

    public ConstraintType getType() {
        return Type;
    }

    public void set(T newValue) {
        Value = newValue;
    }

    public void update(Constraint<T> newConstraint) {
        Type = newConstraint.Type;
        Value = newConstraint.Value;
    }
}
