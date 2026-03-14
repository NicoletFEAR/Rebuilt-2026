package frc.robot.data.controller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.architecture.StateSubsystem;
import frc.robot.data.controller.ControllerIO.ControllerIOInputs;
import frc.robot.data.controller.ControllerIO.Type;
import frc.robot.util.Container;
import frc.robot.util.IOProcessor;
import frc.robot.util.SubsystemIOProcessor;

public class Controller extends StateSubsystem<Controller.State> {
    private final String m_name;
    private final Container<ControllerIO> m_io;
    private final ControllerIOInputsAutoLogged m_inputs = new ControllerIOInputsAutoLogged();
    private Type m_ioType = Type.NONE;
    private final Alert m_missingIO;
    private final Alert m_unrecognizedIO = new Alert("", AlertType.kWarning);
    private DesiredState m_desiredState = DesiredState.IDLE;
    private State m_state = State.IDLE;

    private final int m_port;

    public Controller(String name, int port, IOProcessor processor) {
        m_name = name;
        m_port = port;
        m_missingIO = new Alert(
            String.format("%s disconnected. (port %d)", m_name, m_port),
            AlertType.kWarning
        );

        m_io = new Container<ControllerIO>(chooseIO());

        processor.add(
            new SubsystemIOProcessor<ControllerIOInputs>(
                m_io,
                m_inputs
            )
        );

        createIOChangeTriggers();
    }

    private boolean shouldChangeIO() {
        boolean result = false;

        synchronized (m_inputs) {
            if (!m_ioType.equals(m_inputs.ControllerType)) {
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

    private ControllerIO chooseIO() {
        synchronized (m_inputs) {
            return switch (m_inputs.ControllerType) {
                case KEYBOARD_0 -> {
                    m_missingIO.set(false);
                    m_unrecognizedIO.set(false);
                    yield new ControllerIOKeyboard0(m_port);
                }
                case KEYBOARD_1 -> {
                    m_missingIO.set(false);
                    m_unrecognizedIO.set(false);
                    yield new ControllerIOKeyboard1(m_port);
                }
                case KEYBOARD_2 -> {
                    m_missingIO.set(false);
                    m_unrecognizedIO.set(false);
                    yield new ControllerIOKeyboard2(m_port);
                }

                case PS_4 -> {
                    m_missingIO.set(false);
                    m_unrecognizedIO.set(false);
                    yield new ControllerIOPS4(m_port);
                }
                case PS_5 -> {
                    m_missingIO.set(false);
                    m_unrecognizedIO.set(false);
                    yield new ControllerIOPS5(m_port);
                }
                case XBOX -> {
                    m_missingIO.set(false);
                    m_unrecognizedIO.set(false);
                    yield new ControllerIOXbox(m_port);
                }

                case NONE -> {
                    m_missingIO.set(true);
                    m_unrecognizedIO.set(false);
                    yield new ControllerIONone(m_port);
                }

                case UNRECOGNIZED -> {
                    m_missingIO.set(false);
                    m_unrecognizedIO.setText(
                        String.format("Unrecognized controller type. (port %d)", m_port)
                    );
                    m_unrecognizedIO.set(true);
                    yield new ControllerIONone(m_port);
                }
            };
        }
    }

    public static enum DesiredState {
        IDLE,
        RUMBLE,
    }

    public static enum State {
        IDLE,
        RUMBLING,
    }

    public boolean circle() {
        synchronized (m_inputs) {
            return m_inputs.Circle;
        }
    }

    public boolean create() {
        synchronized (m_inputs) {
            return m_inputs.Create;
        }
    }

    public boolean cross() {
        synchronized (m_inputs) {
            return m_inputs.Cross;
        }
    }

    public boolean leftBumper() {
        synchronized (m_inputs) {
            return m_inputs.LeftBumper;
        }
    }

    public boolean leftTrigger() {
        synchronized (m_inputs) {
            return m_inputs.LeftTrigger > 0.5;
        }
    }

    public boolean left() {
        synchronized (m_inputs) {
            return m_inputs.Left;
        }
    }

    public double leftX() {
        synchronized (m_inputs) {
            return m_inputs.LeftX;
        }
    }

    public double leftY() {
        synchronized (m_inputs) {
            return m_inputs.LeftY;
        }
    }

    public boolean rightBumper() {
        synchronized (m_inputs) {
            return m_inputs.RightBumper;
        }
    }

    public boolean rightTrigger() {
        synchronized (m_inputs) {
            return m_inputs.RightTrigger > 0.5;
        }
    }

    public boolean right() {
        synchronized (m_inputs) {
            return m_inputs.Right;
        }
    }

    public double rightX() {
        synchronized (m_inputs) {
            return m_inputs.RightX;
        }
    }

    public double rightY() {
        synchronized (m_inputs) {
            return m_inputs.RightY;
        }
    }

    public boolean square() {
        synchronized (m_inputs) {
            return m_inputs.Square;
        }
    }

    public boolean triangle() {
        synchronized (m_inputs) {
            return m_inputs.Triangle;
        }
    }

    public Command idle() {
        return Commands.runOnce(() -> m_desiredState = DesiredState.IDLE);
    }

    public Command rumble() {
        return Commands.runOnce(() -> m_desiredState = DesiredState.RUMBLE);
    }
    
    @Override
    protected State updateState() {
        return switch (m_desiredState) {
            case IDLE -> State.IDLE;
            case RUMBLE -> State.RUMBLING;
        };
    }

    @Override
    protected void applyState() {
        switch (m_state) {
            case IDLE -> {
                synchronized (m_io) {
                    m_io.get().setRumble(0.0);
                }
            }
            case RUMBLING -> {
                synchronized (m_io) {
                    m_io.get().setRumble(1.0);
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
