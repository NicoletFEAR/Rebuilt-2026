package frc.robot.io.motor;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CANId;

public class MotorIOSparkMax extends MotorIO {
    private final RelativeEncoder encoder;
    private final SparkMax motor;

    public MotorIOSparkMax(CANId id, MotorType type) {
        super(id, type);
        motor = new SparkMax(id.getDevice(), type);
        SparkMaxConfig configuration = new SparkMaxConfig();
        configuration
            .inverted(false)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        motor.configure(configuration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = motor.getEncoder();
    }

    @Override
    public void applyVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public MotorState updateState() {
        m_state.CurrentIdentity = MotorIdentity.SPARK_MAX;
        m_state.Position = Rotations.of(encoder.getPosition());
        m_state.ProperIdentity = motor.hasActiveFault() ? MotorIdentity.NONE : MotorIdentity.SPARK_MAX;
        m_state.Velocity = RotationsPerSecond.of(encoder.getVelocity());
        m_state.Voltage = Volts.of(motor.getBusVoltage());
        return m_state;
    }
}
