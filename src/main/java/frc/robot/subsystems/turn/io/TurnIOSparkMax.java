package frc.robot.subsystems.turn.io;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.turn.TurnIdentity;
import frc.robot.subsystems.turn.TurnName;
import frc.robot.subsystems.turn.TurnState;
import frc.robot.util.Configurator;

public class TurnIOSparkMax extends TurnIO {
    private final StatusSignal<Angle> m_absolutePosition;
    private final StatusSignal<AngularVelocity> m_velocity;

    public TurnIOSparkMax(TurnName name, DriveConstants driveConstants) {
        super(name, driveConstants);

        SparkMaxConfig motorConfiguration = new SparkMaxConfig();

        motorConfiguration
            .inverted(true)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);

        motorConfiguration.encoder.positionConversionFactor(1.0 / m_driveConstants.kTurnGearRatio);

        CANcoderConfiguration absoluteEncoderConfiguration = new CANcoderConfiguration();
        absoluteEncoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        absoluteEncoderConfiguration.MagnetSensor.MagnetOffset = m_name.getAbsoluteEncoderOffset(m_driveConstants).in(Rotations);
        absoluteEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        Configurator.configure(m_absoluteEncoder, absoluteEncoderConfiguration);

        m_absolutePosition = m_absoluteEncoder.getAbsolutePosition();
        m_velocity = m_absoluteEncoder.getVelocity();
        m_motor.getEncoder().setPosition(m_absolutePosition.getValueAsDouble());
    }

    @Override
    public void applyVoltage(Voltage voltage) {
        m_motor.setVoltage(voltage);
    }

    @Override
    public TurnState updateState() {
        BaseStatusSignal.refreshAll(
            m_absolutePosition,
            m_velocity
        );

        m_state.CurrentIdentity = TurnIdentity.SPARK_MAX;
        m_state.Name = m_name;
        m_state.Position = m_absolutePosition.getValue();
        m_state.ProperIdentity = Robot.isSimulation() ? TurnIdentity.SPARK_MAX_SIMULATED :
            m_motor.hasActiveFault() ? TurnIdentity.NONE : TurnIdentity.SPARK_MAX;
        m_state.Velocity = m_velocity.getValue();
        return m_state;
    }
}
