package frc.robot.io.motor;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import frc.robot.io.base.IO;
import frc.robot.io.motor.MotorValues.MotorConfiguration;

public abstract class MotorIO extends IO<MotorState, MotorIO, MotorIdentity> {
    private final MotorConfiguration configuration;

    public MotorIO(MotorConfiguration configuration) {
        this.configuration = configuration;
        state = new MotorState();
    }

    public void applyVoltage(Voltage voltage) {}

    @Override
    public MotorState updateState() {
        state.CurrentIdentity = MotorIdentity.NONE;

        if (Robot.isSimulation()) {
            state.ProperIdentity = MotorIdentity.TALON_FX_SIMULATED;
        } else {
            TalonFX talon = new TalonFX(configuration.id().device(), configuration.id().bus());

            if (talon.isAlive()) {
                talon.close();
                state.ProperIdentity = MotorIdentity.TALON_FX;
            } else {
                talon.close();
                SparkMax sparkMax = new SparkMax(configuration.id().device(), MotorType.kBrushless);

                if (sparkMax.hasActiveFault()) {
                    sparkMax.close();
                    state.ProperIdentity = MotorIdentity.NONE;
                } else {
                    sparkMax.close();
                    state.ProperIdentity = MotorIdentity.SPARK_MAX;
                }
            }
        }

        state.Velocity = RadiansPerSecond.of(0.0);
        state.Voltage = Volts.of(0.0);
        return state;
    }

    @Override
    public MotorIO getProperIO() {
        return switch (state.ProperIdentity) {
            case NONE -> new MotorIONone(configuration);
            case SPARK_MAX -> new MotorIOSparkMax(configuration);
            case SPARK_MAX_SIMULATED -> new MotorIOSparkMaxSimulated(configuration);
            case TALON_FX -> new MotorIOTalonFX(configuration);
            case TALON_FX_SIMULATED -> new MotorIOTalonFXSimulated(configuration);
        };
    }
}
