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
import frc.robot.io.motor.MotorValues.MotorConfiguration;
import frc.robot.io.motor.MotorValues.MotorInversion;

public class MotorIOSparkMax extends MotorIO {
    private final RelativeEncoder encoder;
    private final SparkMax motor;

    public MotorIOSparkMax(MotorConfiguration configuration) {
        super(configuration);
        motor = new SparkMax(configuration.id().device(), MotorType.kBrushless);
        SparkMaxConfig directConfiguration = new SparkMaxConfig();

        IdleMode idleMode = switch (configuration.neutralMode()) {
            case BRAKE -> IdleMode.kBrake;
            case COAST -> IdleMode.kCoast;
        };

        directConfiguration
            .inverted(configuration.inversion() == MotorInversion.COUNTER_CLOCKWISE_IS_POSITIVE)
            .idleMode(idleMode);

        if (configuration.statorCurrentLimit().isPresent()) {
            directConfiguration.smartCurrentLimit(Math.toIntExact(Math.round(configuration.statorCurrentLimit().get())));
        }

        motor.configure(directConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = motor.getEncoder();
    }

    @Override
    public void applyVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public MotorState updateState() {
        state.CurrentIdentity = MotorIdentity.SPARK_MAX;
        state.Position = Rotations.of(encoder.getPosition());
        state.ProperIdentity = motor.hasActiveFault() ? MotorIdentity.NONE : MotorIdentity.SPARK_MAX;
        state.Velocity = RotationsPerSecond.of(encoder.getVelocity());
        state.Voltage = Volts.of(motor.getBusVoltage());
        return state;
    }
}
