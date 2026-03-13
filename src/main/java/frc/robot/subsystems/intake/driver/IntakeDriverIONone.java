package frc.robot.subsystems.intake.driver;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import frc.robot.util.CANId;

public class IntakeDriverIONone extends IntakeDriverIO {
    private final TalonFX m_motor;

    private final boolean m_configured = true;
    private Type m_intakeDriverType;
    private final Current m_statorCurrent = Amps.of(0.0);
    private final Current m_supplyCurrent = Amps.of(0.0);
    private final Temperature m_temperature = Celsius.of(20.0);
    private final AngularVelocity m_velocity = RotationsPerSecond.of(0.0);
    private final Voltage m_voltage = Volts.of(0.0);

    public IntakeDriverIONone(CANId id) {
        m_motor = new TalonFX(id.getDevice(), id.getBus());
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {}

    @Override
    public void refreshData() {
        m_intakeDriverType = Robot.isSimulation() ? Type.TALON_FX_SIMULATED : m_motor.isAlive() ? Type.TALON_FX : Type.NONE;
    }

    @Override
    public void updateInputs(IntakeDriverIOInputs inputs) {
        inputs.Configured = m_configured;
        inputs.IntakeDriverType = m_intakeDriverType;
        inputs.StatorCurrent = m_statorCurrent;
        inputs.SupplyCurrent = m_supplyCurrent;
        inputs.Temperature = m_temperature;
        inputs.Velocity = m_velocity;
        inputs.Voltage = m_voltage;
    }
}
