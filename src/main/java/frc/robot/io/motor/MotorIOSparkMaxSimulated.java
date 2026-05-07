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
import frc.robot.io.motor.MotorValues.MotorConfiguration;
import frc.robot.io.motor.MotorValues.MotorInversion;

public class MotorIOSparkMaxSimulated extends MotorIO {
    private final SparkMax motor;
    private final DCMotorSim motorModel;
    private final SparkMaxSim motorSimulation;

    public MotorIOSparkMaxSimulated(MotorConfiguration configuration) {
        super(configuration);
        motorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1),
                0.004,
                1.0
            ),
            DCMotor.getNEO(1)
        );
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
        motorSimulation = new SparkMaxSim(motor, DCMotor.getNEO(1));
    }

    @Override
    public void applyVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public MotorState updateState() {
        motorModel.setInputVoltage(motorSimulation.getAppliedOutput() * RobotController.getBatteryVoltage());
        motorModel.update(Constants.LOOP_PERIOD);

        motorSimulation.iterate(
            motorModel.getAngularVelocity().in(RotationsPerSecond) * 60.0,
            RobotController.getBatteryVoltage(),
            Constants.LOOP_PERIOD
        );

        state.CurrentIdentity = MotorIdentity.SPARK_MAX_SIMULATED;
        state.Position = Rotations.of(motor.getEncoder().getPosition());
        state.ProperIdentity = motor.hasActiveFault() ? MotorIdentity.NONE : MotorIdentity.SPARK_MAX_SIMULATED;
        state.Velocity = RotationsPerSecond.of(motor.getEncoder().getVelocity());
        state.Voltage = Volts.of(motor.getBusVoltage());
        return state;
    }
}
