package frc.robot.util.constraint;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class BooleanConstraint extends Constraint<Boolean> {
    public BooleanConstraint() {
        Value = false;
    }

    public BooleanConstraintAutoLogged toAutoLogged() {
        BooleanConstraintAutoLogged result = new BooleanConstraintAutoLogged();
        result.update(this);
        return result;
    }
}
