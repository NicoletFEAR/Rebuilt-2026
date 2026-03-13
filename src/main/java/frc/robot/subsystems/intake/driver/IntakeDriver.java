package frc.robot.subsystems.intake.driver;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.architecture.StateSubsystem;
import frc.robot.subsystems.intake.driver.IntakeDriverIO.IntakeDriverIOInputs;
import frc.robot.util.CANId;
import frc.robot.util.Container;
import frc.robot.util.IOProcessor;
import frc.robot.util.SubsystemIOProcessor;

public class IntakeDriver extends StateSubsystem<IntakeDriver.State> {
    private final String m_name;
    private final Container<IntakeDriverIO> m_io;
    private final IntakeDriverIOInputsAutoLogged m_inputs = new IntakeDriverIOInputsAutoLogged();
    private final Alert m_missingIO;
    private final Alert m_unrecognizedIO = new Alert("", AlertType.kError);
    private DesiredState m_desiredState = DesiredState.IDLE;
    private State m_state = State.IDLE;

    private final CANId m_id;

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

        createIOChangeTrigger();
    }

    private boolean shouldChangeIO() {
        return false;
    }

    private void createIOChangeTrigger() {
        new Trigger(this::shouldChangeIO).onTrue(new InstantCommand(() -> {
                synchronized (m_io) {
                    m_io.set(chooseIO());
                }
            }).ignoringDisable(true));
    }

    private IntakeDriverIO chooseIO() {
        return new IntakeDriverIOTalonFX(m_id);
    }

    public static enum DesiredState {
        IDLE,
    }

    public static enum State {
        IDLE,
    }

    @Override
    protected State updateState() {
        return switch (m_desiredState) {
            case IDLE -> State.IDLE;
        };
    }

    @Override
    protected void applyState() {
        switch (m_state) {
            case IDLE -> {
                synchronized (m_io) {
                    m_io.get().setVelocity(RotationsPerSecond.of(0.0));
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
