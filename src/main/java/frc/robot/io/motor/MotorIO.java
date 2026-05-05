package frc.robot.io.motor;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import frc.robot.io.base.IO;
import frc.robot.util.CANId;

public abstract class MotorIO extends IO<MotorState, MotorIO, MotorIdentity> {
    CANId id;
    MotorType type;

    public MotorIO(CANId id, MotorType type) {
        this.id = id;
        this.type = type;
        m_state = new MotorState();
    }

    public void applyVoltage(Voltage voltage) {}

    @Override
    public MotorState updateState() {
        m_state.CurrentIdentity = MotorIdentity.NONE;

        if (Robot.isSimulation()) {
            m_state.ProperIdentity = MotorIdentity.TALON_FX_SIMULATED;
        } else {
            TalonFX talon = new TalonFX(id.getDevice(), id.getBus());

            if (talon.isAlive()) {
                talon.close();
                m_state.ProperIdentity = MotorIdentity.TALON_FX;
            } else {
                talon.close();
                SparkMax sparkMax = new SparkMax(id.getDevice(), type);

                if (sparkMax.hasActiveFault()) {
                    sparkMax.close();
                    m_state.ProperIdentity = MotorIdentity.NONE;
                } else {
                    sparkMax.close();
                    m_state.ProperIdentity = MotorIdentity.SPARK_MAX;
                }
            }
        }

        m_state.Velocity = RadiansPerSecond.of(0.0);
        m_state.Voltage = Volts.of(0.0);
        return m_state;
    }

    @Override
    public MotorIO getProperIO() {
        return switch (m_state.ProperIdentity) {
            case NONE -> new MotorIONone(id, type);
            case SPARK_MAX -> new MotorIOSparkMax(id, type);
            case SPARK_MAX_SIMULATED -> new MotorIOSparkMaxSimulated(id, type);
            case TALON_FX -> new MotorIONone(id, type);
            case TALON_FX_SIMULATED -> new MotorIONone(id, type);
        };
    }
}
