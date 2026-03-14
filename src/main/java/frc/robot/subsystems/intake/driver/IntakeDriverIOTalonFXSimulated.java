package frc.robot.subsystems.intake.driver;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.CANId;
import frc.robot.util.Utils;

public class IntakeDriverIOTalonFXSimulated extends IntakeDriverIO {
    private final TalonFX m_motor;
    private final TalonFXSimState m_motorSim;
    private final DCMotorSim m_motorModel;

    private final StatusSignal<AngularAcceleration> m_acceleration;
    private final boolean m_configured;
    private final StatusSignal<Angle> m_position;
    private final StatusSignal<Current> m_statorCurrent;
    private final StatusSignal<Current> m_supplyCurrent;
    private final StatusSignal<Temperature> m_temperature;
    private final StatusSignal<AngularVelocity> m_velocity;
    private final StatusSignal<Voltage> m_voltage;

    private final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0.0).withSlot(0);
    private final VoltageOut m_setVoltageRequest = new VoltageOut(0.0);

    public IntakeDriverIOTalonFXSimulated(CANId id) {
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

        configuration.Slot0.kP = 0.028981;
        configuration.Slot0.kI = 0.0;
        configuration.Slot0.kD = 0.0;
        configuration.Slot0.kS = 0.0048976;
        configuration.Slot0.kV = 0.018915;
        configuration.Slot0.kA = 0.0050602;

        configuration.MotionMagic.MotionMagicCruiseVelocity = RotationsPerSecond.convertFrom(505.227, RadiansPerSecond);
        configuration.MotionMagic.MotionMagicAcceleration = RotationsPerSecondPerSecond.convertFrom(801.734, RadiansPerSecondPerSecond);

        m_configured = Utils.configure(m_motor, configuration);

        m_motorSim = m_motor.getSimState();
        m_motorSim.setMotorType(MotorType.KrakenX60);
        m_motorSim.Orientation = ChassisReference.Clockwise_Positive;
        m_motorModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 0.6), DCMotor.getKrakenX60(1));

        m_acceleration = m_motor.getAcceleration();
        m_position = m_motor.getPosition();
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
    public void setVoltage(Voltage voltage) {
        m_motor.setControl(m_setVoltageRequest.withOutput(voltage));
    }

    @Override
    public void refreshData() {
        m_motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_motorModel.setInputVoltage(m_motorSim.getMotorVoltage());
        m_motorModel.update(Constants.kLoopTime);
        m_motorSim.setRawRotorPosition(m_motorModel.getAngularPosition().times(0.6));
        m_motorSim.setRotorVelocity(m_motorModel.getAngularVelocity().times(0.6));
        m_motorSim.setRotorAcceleration(m_motorModel.getAngularAcceleration().times(0.6));

        BaseStatusSignal.refreshAll(
            m_acceleration,
            m_position,
            m_statorCurrent,
            m_supplyCurrent,
            m_temperature,
            m_velocity,
            m_voltage
        );
    }

    @Override
    public void updateInputs(IntakeDriverIOInputs inputs) {
        inputs.Acceleration = m_acceleration.getValue();
        inputs.Configured = m_configured;
        inputs.IntakeDriverType = Type.TALON_FX_SIMULATED;
        inputs.Position = m_position.getValue();
        inputs.StatorCurrent = m_statorCurrent.getValue();
        inputs.SupplyCurrent = m_supplyCurrent.getValue();
        inputs.Temperature = m_temperature.getValue();
        inputs.Velocity = m_velocity.getValue();
        inputs.Voltage = m_voltage.getValue();
    }
}
