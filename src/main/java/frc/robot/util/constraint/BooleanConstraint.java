package frc.robot.util.constraint;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class BooleanConstraint {
    protected ConstraintType Type = ConstraintType.IGNORE;
    protected boolean Value = false;

    public boolean get() {
        return Value;
    }

    public ConstraintType getType() {
        return Type;
    }

    public void set(boolean newValue) {
        Value = newValue;
    }

    public void update(BooleanConstraint newConstraint) {
        Type = newConstraint.Type;
        Value = newConstraint.Value;
    }

    public BooleanConstraintAutoLogged toAutoLogged() {
        BooleanConstraintAutoLogged result = new BooleanConstraintAutoLogged();
        result.update(this);
        return result;
    }
}
