package frc.robot.data.controller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.architecture.StateSubsystem;
import frc.robot.data.controller.ControllerIO.ControllerIOInputs;
import frc.robot.util.Container;
import frc.robot.util.IOProcessor;
import frc.robot.util.SubsystemIOProcessor;

public class Controller extends StateSubsystem<Controller.State> {
    private final String m_name;
    private final Container<ControllerIO> m_io;
    private final ControllerIOInputsAutoLogged m_inputs = new ControllerIOInputsAutoLogged();
    private final Alert m_missingIO;
    private final Alert m_unrecognizedIO = new Alert("", AlertType.kWarning);
    private DesiredState m_desiredState = DesiredState.IDLE;
    private State m_state = State.IDLE;

    private final int m_port;
    private String m_joystickName;

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

        createIOChangeTrigger();
    }

    private boolean shouldChangeIO() {
        return !m_joystickName.equals(DriverStation.getJoystickName(m_port));
    }

    private void createIOChangeTrigger() {
        new Trigger(this::shouldChangeIO).onTrue(new InstantCommand(() -> {
                synchronized (m_io) {
                    m_io.set(chooseIO());
                }
            }).ignoringDisable(true));
    }

    private ControllerIO chooseIO() {
        m_joystickName = DriverStation.getJoystickName(m_port);

        return switch (m_joystickName) {
            case "Keyboard 0" -> {
                m_missingIO.set(false);
                m_unrecognizedIO.set(false);
                yield new ControllerIOKeyboard0(m_port);
            }
            case "Keyboard 1" -> {
                m_missingIO.set(false);
                m_unrecognizedIO.set(false);
                yield new ControllerIOKeyboard1(m_port);
            }
            case "Keyboard 2" -> {
                m_missingIO.set(false);
                m_unrecognizedIO.set(false);
                yield new ControllerIOKeyboard2(m_port);
            }

            case "Wireless Controller" -> {
                m_missingIO.set(false);
                m_unrecognizedIO.set(false);
                yield new ControllerIOPS4(m_port);
            }
            case "DualSense Wireless Controller" -> {
                m_missingIO.set(false);
                m_unrecognizedIO.set(false);
                yield new ControllerIOPS5(m_port);
            }
            case "Controller (Gamepad F310)", "Xbox Controller" -> {
                m_missingIO.set(false);
                m_unrecognizedIO.set(false);
                yield new ControllerIOXbox(m_port);
            }

            case "" -> {
                m_missingIO.set(true);
                m_unrecognizedIO.set(false);
                yield new ControllerIONone();
            }

            default -> {
                m_missingIO.set(false);
                m_unrecognizedIO.setText(
                    String.format("Unrecognized controller type: %s. (port %d)", m_joystickName, m_port)
                );
                m_unrecognizedIO.set(true);
                yield new ControllerIONone();
            }
        };
    }

    public static enum DesiredState {
        IDLE,
        RUMBLING,
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

    public InstantCommand idle() {
        return new InstantCommand(() -> m_desiredState = DesiredState.IDLE);
    }

    public InstantCommand rumble() {
        return new InstantCommand(() -> m_desiredState = DesiredState.RUMBLING);
    }
    
    @Override
    protected State updateState() {
        return switch (m_desiredState) {
            case IDLE -> State.IDLE;
            case RUMBLING -> State.RUMBLING;
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
