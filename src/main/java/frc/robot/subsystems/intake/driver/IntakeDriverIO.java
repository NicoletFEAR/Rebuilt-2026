package frc.robot.subsystems.intake.driver;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.architecture.SubsystemIO;

public abstract class IntakeDriverIO implements SubsystemIO<IntakeDriverIO.IntakeDriverIOInputs> {
    @AutoLog
    public static class IntakeDriverIOInputs {
        public boolean Configured;
        public Type IntakeDriverType;
        public Current StatorCurrent;
        public Current SupplyCurrent;
        public Temperature Temperature;
        public AngularVelocity Velocity;
        public Voltage Voltage;
    }

    public abstract void setVelocity(AngularVelocity velocity);

    public static enum Type {
        NONE,
        SIMULATED,
        TALON_FX,
    }
}
