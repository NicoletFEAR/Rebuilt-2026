package frc.robot.subsystems.intake.driver;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.architecture.StateSubsystem;
import frc.robot.subsystems.intake.driver.IntakeDriverIO.IntakeDriverIOInputs;
import frc.robot.subsystems.intake.driver.IntakeDriverIO.Type;
import frc.robot.util.CANId;
import frc.robot.util.Container;
import frc.robot.util.IOProcessor;
import frc.robot.util.SubsystemIOProcessor;

public class IntakeDriver extends StateSubsystem<IntakeDriver.State> {
    private final String m_name;
    private final Container<IntakeDriverIO> m_io;
    private final IntakeDriverIOInputsAutoLogged m_inputs = new IntakeDriverIOInputsAutoLogged();
    private Type m_ioType = Type.NONE;
    private final Alert m_missingIO;
    private DesiredState m_desiredState = DesiredState.IDLE;
    private State m_state = State.IDLE;

    private final CANId m_id;
    private Voltage m_runningVoltage = Volts.of(0.0);

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1.0).per(Seconds),
            Volts.of(7.0),
            Seconds.of(5),
            (state) -> Logger.recordOutput("SysIdState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (voltage) -> m_runningVoltage = voltage,
            null,
            this
        )
    );

    public IntakeDriver(String name, CANId driverId, IOProcessor processor) {
        m_name = name;
        m_id = driverId;
        m_missingIO = new Alert(
            String.format("%s disconnected. (Device %d on CAN bus %s)", m_name, m_id.getDevice(), m_id.getBus().getName()),
            AlertType.kError
        );

        m_io = new Container<IntakeDriverIO>(chooseIO());

        processor.add(
            new SubsystemIOProcessor<IntakeDriverIOInputs>(
                m_io,
                m_inputs
            )
        );

        createIOChangeTriggers();
    }
    
    private boolean shouldChangeIO() {
        boolean result = false;

        synchronized (m_inputs) {
            if (m_ioType != m_inputs.IntakeDriverType) {
                result = true;
            }
        }

        return result;
    }

    private void createIOChangeTriggers() {
        new Trigger(this::shouldChangeIO).onTrue(Commands.runOnce(() -> {
                synchronized (m_io) {
                    m_io.set(chooseIO());
                }
            }).ignoringDisable(true));

        new Trigger(DriverStation::isDSAttached).onTrue(Commands.runOnce(() -> {
                synchronized (m_io) {
                    m_io.set(chooseIO());
                }
            }).ignoringDisable(true));
    }

    private IntakeDriverIO chooseIO() {
        synchronized (m_inputs) {
            return switch (m_inputs.IntakeDriverType) {
                case TALON_FX -> {
                    m_missingIO.set(false);
                    m_ioType = Type.TALON_FX;
                    yield new IntakeDriverIOTalonFX(m_id);
                }

                case TALON_FX_SIMULATED -> {
                    m_missingIO.set(false);
                    m_ioType = Type.TALON_FX_SIMULATED;
                    yield new IntakeDriverIOTalonFXSimulated(m_id);
                }

                case NONE -> {
                    m_missingIO.set(true);
                    m_ioType = Type.NONE;
                    yield new IntakeDriverIONone(m_id);
                }
            };
        }
    }

    public static enum DesiredState {
        IDLE,
        INTAKE,
        RUN_VOLTAGE,
    }

    public static enum State {
        IDLE,
        INTAKING,
        RUNNING_VOLTAGE,
    }

    public Command idle() {
        return Commands.runOnce(() -> m_desiredState = DesiredState.IDLE);
    }

    public Command intake() {
        return Commands.runOnce(() -> m_desiredState = DesiredState.INTAKE);
    }

    public Command runVoltage(Voltage voltage) {
        m_runningVoltage = voltage;
        return Commands.runOnce(() -> m_desiredState = DesiredState.RUN_VOLTAGE);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Commands.runOnce(() -> m_desiredState = DesiredState.RUN_VOLTAGE)
            .andThen(m_sysIdRoutine.quasistatic(direction))
            .andThen(Commands.runOnce(() -> m_desiredState = DesiredState.IDLE));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return Commands.runOnce(() -> m_desiredState = DesiredState.RUN_VOLTAGE)
            .andThen(m_sysIdRoutine.dynamic(direction))
            .andThen(Commands.runOnce(() -> m_desiredState = DesiredState.IDLE));
    }

    @Override
    protected State updateState() {
        return switch (m_desiredState) {
            case IDLE -> State.IDLE;
            case INTAKE -> State.INTAKING;
            case RUN_VOLTAGE -> State.RUNNING_VOLTAGE;
        };
    }

    @Override
    protected void applyState() {
        switch (m_state) {
            case IDLE -> {
                synchronized (m_io) {
                    m_io.get().setVelocity(RadiansPerSecond.of(0.0));
                }
            }

            case INTAKING -> {
                synchronized (m_io) {
                    m_io.get().setVelocity(RadiansPerSecond.of(600.0));
                }
            }

            case RUNNING_VOLTAGE -> {
                synchronized (m_io) {
                    m_io.get().setVoltage(m_runningVoltage);
                }
            }
        }
    }

    @Override
    public void periodic() {
        synchronized (m_inputs) {
            Logger.processInputs(m_name, m_inputs);
        }

        m_state = updateState();
        Logger.recordOutput(m_name + "/DesiredState", m_desiredState);
        Logger.recordOutput(m_name + "/State", m_state);
        applyState();
    }
}
