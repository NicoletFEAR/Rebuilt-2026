package frc.robot.subsystems.turn.inputs;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.base.Inputs;
import frc.robot.subsystems.turn.TurnIdentity;
import frc.robot.subsystems.turn.TurnName;
import frc.robot.subsystems.turn.TurnState;
import frc.robot.util.CANId;

public abstract class TurnInputs extends Inputs<TurnState, TurnInputs, TurnName, TurnIdentity> {
    protected final DriveConstants m_driveConstants;
    protected final SparkMax m_motor;
    protected final CANcoder m_absoluteEncoder;

    public TurnInputs(TurnName name, DriveConstants driveConstants) {
        super(name);
        m_driveConstants = driveConstants;
        m_motor = new SparkMax(m_name.getMotorId().getDevice(), MotorType.kBrushless);

        CANId encoderId = m_name.getAbsoluteEncoderId();
        m_absoluteEncoder = new CANcoder(encoderId.getDevice(), encoderId.getBus());
        m_state = new TurnState();
    }

    public void applyVoltage(Voltage voltage) {};

    @Override
    public TurnState updateState() {
        m_state.CurrentIdentity = TurnIdentity.NONE;
        m_state.Name = m_name;
        m_state.ProperIdentity = Robot.isSimulation() ? TurnIdentity.SPARK_MAX_SIMULATED :
            m_motor.hasActiveFault() || !m_absoluteEncoder.isConnected() ? TurnIdentity.NONE : TurnIdentity.SPARK_MAX;
        m_state.Velocity = RadiansPerSecond.of(0.0);
        m_state.Voltage = Volts.of(0.0);
        return m_state;
    }

    @Override
    public TurnInputs getInputs() {
        return switch (m_state.ProperIdentity) {
            case NONE -> new TurnInputsNone(m_name, m_driveConstants);
            case SPARK_MAX -> new TurnInputsSparkMax(m_name, m_driveConstants);
            case SPARK_MAX_SIMULATED -> new TurnInputsSparkMaxSimulated(m_name, m_driveConstants);
        };
    }
}
