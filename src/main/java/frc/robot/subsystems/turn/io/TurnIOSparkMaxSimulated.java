package frc.robot.subsystems.turn.io;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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
    private CANcoderSimState m_absoluteEncoderSimulation;
    private final RelativeEncoder m_relativeEncoder;
    private final StatusSignal<Angle> m_absolutePosition;

    public TurnIOSparkMaxSimulated(TurnName name, DriveConstants driveConstants) {
        super(name, driveConstants);

        m_motorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1),
                0.001,
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
        m_relativeEncoder = m_motor.getEncoder();

        CANcoderConfiguration absoluteEncoderConfiguration = new CANcoderConfiguration();
        absoluteEncoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        absoluteEncoderConfiguration.MagnetSensor.MagnetOffset = m_name.getAbsoluteEncoderOffset(m_driveConstants).in(Rotations);
        absoluteEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        Configurator.configure(m_absoluteEncoder, absoluteEncoderConfiguration);

        m_absolutePosition = m_absoluteEncoder.getAbsolutePosition();
        m_relativeEncoder.setPosition(m_absolutePosition.getValueAsDouble());
    }

    @Override
    public void applyVoltage(Voltage voltage) {
        m_motor.setVoltage(voltage);
    }

    @Override
    public TurnState updateState() {
        m_motorModel.setInput(m_motorSimulation.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_motorModel.update(Constants.kLoopPeriod);

        m_motorSimulation.iterate(
            m_motorModel.getAngularVelocity().in(RotationsPerSecond) * 60.0,
            RobotController.getBatteryVoltage(),
            Constants.kLoopPeriod
        );

        m_absoluteEncoderSimulation.setSupplyVoltage(Volts.of(RobotController.getBatteryVoltage()));
        m_absoluteEncoderSimulation.setRawPosition(m_motorModel.getAngularPosition());
        m_absoluteEncoderSimulation.setVelocity(m_motorModel.getAngularVelocity());

        m_absolutePosition.refresh();

        m_state.CurrentIdentity = TurnIdentity.SPARK_MAX;
        m_state.Name = m_name;
        m_state.Position = m_absolutePosition.getValue();
        m_state.ProperIdentity = Robot.isSimulation() ? TurnIdentity.SPARK_MAX_SIMULATED :
            m_motor.hasActiveFault() ? TurnIdentity.NONE : TurnIdentity.SPARK_MAX;
        m_state.Velocity = RotationsPerSecond.of(m_relativeEncoder.getVelocity());
        return m_state;
    }
}
