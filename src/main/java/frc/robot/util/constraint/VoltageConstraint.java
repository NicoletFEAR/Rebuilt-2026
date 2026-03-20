package frc.robot.util.constraint;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

@AutoLog
public class VoltageConstraint extends Constraint<Voltage> {
    public VoltageConstraint() {
        Value = Volts.of(0.0);
    }

    public VoltageConstraintAutoLogged toAutoLogged() {
        VoltageConstraintAutoLogged result = new VoltageConstraintAutoLogged();
        result.update(this);
        return result;
    }
}
