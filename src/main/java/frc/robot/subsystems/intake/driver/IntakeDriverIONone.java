package frc.robot.subsystems.intake.driver;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kelvin;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeDriverIONone implements IntakeDriverIO {
    private final boolean m_configured = true;
    private final Current m_statorCurrent = Amps.of(0.0);
    private final Current m_supplyCurrent = Amps.of(0.0);
    private final Temperature m_temperature = Kelvin.of(0.0);
    private final AngularVelocity m_velocity = RotationsPerSecond.of(0.0);
    private final Voltage m_voltage = Volts.of(0.0);

    @Override
    public void setVelocity(AngularVelocity velocity) {}

    @Override
    public void refreshData() {}

    @Override
    public void updateInputs(IntakeDriverIOInputs inputs) {
        inputs.Configured = m_configured;
        inputs.StatorCurrent = m_statorCurrent;
        inputs.SupplyCurrent = m_supplyCurrent;
        inputs.Temperature = m_temperature;
        inputs.Velocity = m_velocity;
        inputs.Voltage = m_voltage;
    }
}
