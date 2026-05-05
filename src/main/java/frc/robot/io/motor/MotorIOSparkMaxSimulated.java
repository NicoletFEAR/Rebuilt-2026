package frc.robot.io.motor;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;
import frc.robot.util.CANId;

public class MotorIOSparkMaxSimulated extends MotorIO {
    private final SparkMax motor;
    private final DCMotorSim motorModel;
    private final SparkMaxSim motorSimulation;

    public MotorIOSparkMaxSimulated(CANId id, MotorType type) {
        super(id, type);
        motorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1),
                0.004,
                1.0
            ),
            DCMotor.getNEO(1)
        );
        motor = new SparkMax(id.getDevice(), type);
        SparkMaxConfig configuration = new SparkMaxConfig();
        configuration
            .inverted(false)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        motor.configure(configuration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorSimulation = new SparkMaxSim(motor, DCMotor.getNEO(1));
    }

    @Override
    public void applyVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public MotorState updateState() {
        motorModel.setInputVoltage(motorSimulation.getAppliedOutput() * RobotController.getBatteryVoltage());
        motorModel.update(Constants.kLoopPeriod);

        motorSimulation.iterate(
            motorModel.getAngularVelocity().in(RotationsPerSecond) * 60.0,
            RobotController.getBatteryVoltage(),
            Constants.kLoopPeriod
        );

        m_state.CurrentIdentity = MotorIdentity.SPARK_MAX;
        m_state.Position = Rotations.of(motor.getEncoder().getPosition());
        m_state.ProperIdentity = motor.hasActiveFault() ? MotorIdentity.NONE : MotorIdentity.SPARK_MAX;
        m_state.Velocity = RotationsPerSecond.of(motor.getEncoder().getVelocity());
        m_state.Voltage = Volts.of(motor.getBusVoltage());
        return m_state;
    }
}
