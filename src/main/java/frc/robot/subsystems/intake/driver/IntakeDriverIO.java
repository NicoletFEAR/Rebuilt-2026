package frc.robot.subsystems.intake.driver;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.architecture.SubsystemIO;

public interface IntakeDriverIO extends SubsystemIO<IntakeDriverIO.IntakeDriverIOInputs> {
    @AutoLog
    public static class IntakeDriverIOInputs {
        public boolean Configured;
        public Current StatorCurrent;
        public Current SupplyCurrent;
        public Temperature Temperature;
        public AngularVelocity Velocity;
        public Voltage Voltage;
    }

    void setVelocity(AngularVelocity velocity);
}
