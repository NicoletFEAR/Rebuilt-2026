package frc.robot.subsystems.intake.driver;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kelvin;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.architecture.SubsystemIO;

public abstract class IntakeDriverIO implements SubsystemIO<IntakeDriverIO.IntakeDriverIOInputs> {
    @AutoLog
    public static class IntakeDriverIOInputs {
        public AngularAcceleration Acceleration = RotationsPerSecondPerSecond.of(0.0);
        public boolean Configured = false;
        public Type IntakeDriverType = Type.NONE;
        public Angle Position = Rotations.of(0.0);
        public Current StatorCurrent = Amps.of(0.0);
        public Current SupplyCurrent = Amps.of(0.0);
        public Temperature Temperature = Kelvin.of(293.15);
        public AngularVelocity Velocity = RotationsPerSecond.of(0.0);
        public Voltage Voltage = Volts.of(0.0);
    }

    public abstract void setVelocity(AngularVelocity velocity);

    public abstract void setVoltage(Voltage voltage);

    public static enum Type {
        NONE,
        TALON_FX_SIMULATED,
        TALON_FX,
    }
}
