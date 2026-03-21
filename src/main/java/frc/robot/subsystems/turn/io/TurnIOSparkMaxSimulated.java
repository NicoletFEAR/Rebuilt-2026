package frc.robot.subsystems.turn.io;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.turn.TurnIdentity;
import frc.robot.subsystems.turn.TurnName;
import frc.robot.subsystems.turn.TurnState;
import frc.robot.util.Configurator;

public class TurnIOSparkMaxSimulated extends TurnIO {
    private final SparkMaxSim m_motorSimulation;
    private final DCMotorSim m_motorModel;
    private final CANcoderSimState m_absoluteEncoderSimulation;
    private final StatusSignal<Angle> m_absolutePosition;
    private final StatusSignal<AngularVelocity> m_velocity;

    public TurnIOSparkMaxSimulated(TurnName name, DriveConstants driveConstants) {
        super(name, driveConstants);

        m_motorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1),
                0.004,
                m_driveConstants.kTurnGearRatio
            ),
            DCMotor.getNeo550(1)
        );

        SparkMaxConfig motorConfiguration = new SparkMaxConfig();

        motorConfiguration
            .inverted(true)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);

        motorConfiguration.encoder.positionConversionFactor(1.0 / m_driveConstants.kTurnGearRatio);

        m_motor.configure(motorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motorSimulation = new SparkMaxSim(m_motor, DCMotor.getNeo550(1));

        CANcoderConfiguration absoluteEncoderConfiguration = new CANcoderConfiguration();
        absoluteEncoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        absoluteEncoderConfiguration.MagnetSensor.MagnetOffset = m_name.getAbsoluteEncoderOffset(m_driveConstants).in(Rotations);
        absoluteEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        Configurator.configure(m_absoluteEncoder, absoluteEncoderConfiguration);
        m_absoluteEncoderSimulation = m_absoluteEncoder.getSimState();

        m_absolutePosition = m_absoluteEncoder.getAbsolutePosition();
        m_velocity = m_absoluteEncoder.getVelocity();
        m_motor.getEncoder().setPosition(m_absolutePosition.getValueAsDouble());
    }

    @Override
    public Command applyVoltage(Voltage voltage) {
        return Commands.runOnce(() -> m_motor.setVoltage(voltage));
    }

    @Override
    public TurnState updateState() {
        m_motorModel.setInputVoltage(m_motorSimulation.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_motorModel.update(Constants.kLoopPeriod);

        m_motorSimulation.iterate(
            m_motorModel.getAngularVelocity().in(RotationsPerSecond) * 60.0 * m_driveConstants.kTurnGearRatio,
            RobotController.getBatteryVoltage(),
            Constants.kLoopPeriod
        );

        m_absoluteEncoderSimulation.setSupplyVoltage(Volts.of(RobotController.getBatteryVoltage()));
        m_absoluteEncoderSimulation.setRawPosition(m_motorModel.getAngularPosition());
        m_absoluteEncoderSimulation.setVelocity(m_motorModel.getAngularVelocity());

        BaseStatusSignal.refreshAll(
            m_absolutePosition,
            m_velocity
        );

        m_state.CurrentIdentity = TurnIdentity.SPARK_MAX_SIMULATED;
        m_state.Name = m_name;
        m_state.Position = m_absolutePosition.getValue();
        m_state.ProperIdentity = Robot.isSimulation() ? TurnIdentity.SPARK_MAX_SIMULATED :
            m_motor.hasActiveFault() ? TurnIdentity.NONE : TurnIdentity.SPARK_MAX;
        m_state.Velocity = m_velocity.getValue();
        m_state.Voltage = Volts.of(m_motorSimulation.getAppliedOutput() * RobotController.getBatteryVoltage());
        return m_state;
    }
}
