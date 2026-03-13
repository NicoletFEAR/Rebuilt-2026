package frc.robot.subsystems.intake.driver;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CANId;
import frc.robot.util.Utils;

public class IntakeDriverIOTalonFX implements IntakeDriverIO {
    private final TalonFX m_motor;

    private final boolean m_configured;
    private final StatusSignal<Current> m_statorCurrent;
    private final StatusSignal<Current> m_supplyCurrent;
    private final StatusSignal<Temperature> m_temperature;
    private final StatusSignal<AngularVelocity> m_velocity;
    private final StatusSignal<Voltage> m_voltage;

    private final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0.0).withSlot(0);

    public IntakeDriverIOTalonFX(CANId id) {
        m_motor = new TalonFX(id.getDevice(), id.getBus());
        m_motor.clearStickyFaults();

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configuration.MotorOutput.DutyCycleNeutralDeadband = 0.04;

        configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimit = 120.0;
        configuration.CurrentLimits.SupplyCurrentLimit = 60.0;

        configuration.Feedback.SensorToMechanismRatio = 0.6;

        configuration.Slot0.kP = 1.0;
        configuration.Slot0.kI = 0.0;
        configuration.Slot0.kD = 0.0;
        configuration.Slot0.kS = 0.0;
        configuration.Slot0.kV = 0.0;
        configuration.Slot0.kA = 0.0;

        configuration.MotionMagic.MotionMagicCruiseVelocity = 1.0;
        configuration.MotionMagic.MotionMagicAcceleration = 1.0;

        m_configured = Utils.configure(m_motor, configuration);
        m_statorCurrent = m_motor.getStatorCurrent();
        m_supplyCurrent = m_motor.getSupplyCurrent();
        m_temperature = m_motor.getDeviceTemp();
        m_velocity = m_motor.getRotorVelocity();
        m_voltage = m_motor.getMotorVoltage();
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        m_motor.setControl(m_request.withVelocity(velocity));
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
            m_statorCurrent,
            m_supplyCurrent,
            m_temperature,
            m_velocity,
            m_voltage
        );
    }

    @Override
    public void updateInputs(IntakeDriverIOInputs inputs) {
        inputs.Configured = m_configured;
        inputs.StatorCurrent = m_statorCurrent.getValue();
        inputs.SupplyCurrent = m_supplyCurrent.getValue();
        inputs.Temperature = m_temperature.getValue();
        inputs.Velocity = m_velocity.getValue();
        inputs.Voltage = m_voltage.getValue();
    }
}
