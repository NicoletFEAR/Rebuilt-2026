package frc.robot.subsystems.controller;

import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.architecture.StateSubsystem;
import frc.robot.subsystems.controller.ControllerIO.ControllerIOInputs;
import frc.robot.util.SubsystemIOProcessor;

public class ControllerSubsystem extends StateSubsystem<ControllerSubsystem.State> {
    private final String m_name;
    private final AtomicReference<ControllerIO> m_controllerIO;
    private final ControllerIOInputsAutoLogged m_controllerInputs = new ControllerIOInputsAutoLogged();
    private final Alert m_missingIO;
    private DesiredState m_desiredState = DesiredState.IDLE;
    private State m_state = State.IDLE;

    private final int m_port;
    private String m_joystickName;

    public ControllerSubsystem(String name, int port) {
        m_name = name;
        m_port = port;
        m_missingIO = new Alert(
            String.format("%s disconnected. (port %d)", m_name, m_port),
            AlertType.kWarning
        );

        m_controllerIO = new AtomicReference<ControllerIO>(chooseIO());

        new Thread(
            new SubsystemIOProcessor<ControllerIOInputs>(
                m_controllerIO,
                m_controllerInputs
            )
        ).start();

        new Trigger(this::shouldChangeIO)
            .onTrue(new InstantCommand(() -> m_controllerIO.set(chooseIO())).ignoringDisable(true));
    }

    private boolean shouldChangeIO() {
        return !m_joystickName.equals(DriverStation.getJoystickName(m_port));
    }

    private ControllerIO chooseIO() {
        m_joystickName = DriverStation.getJoystickName(m_port);

        return switch (m_joystickName) {
            case "Keyboard 0": {
                m_missingIO.set(false);
                yield new ControllerIOKeyboard0(m_port);
            }
            case "Keyboard 1": {
                m_missingIO.set(false);
                yield new ControllerIOKeyboard1(m_port);
            }
            case "Keyboard 2": {
                m_missingIO.set(false);
                yield new ControllerIOKeyboard2(m_port);
            }

            case "Wireless Controller": {
                m_missingIO.set(false);
                yield new ControllerIOPS4(m_port);
            }
            case "DualSense Wireless Controller": {
                m_missingIO.set(false);
                yield new ControllerIOPS5(m_port);
            }
            case "Xbox Controller": {
                m_missingIO.set(false);
                yield new ControllerIOXbox(m_port);
            }

            default: {
                m_missingIO.set(true);
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
        synchronized (m_controllerInputs) {
            return m_controllerInputs.Circle;
        }
    }

    public boolean create() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.Create;
        }
    }

    public boolean cross() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.Cross;
        }
    }

    public boolean leftBumper() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.LeftBumper;
        }
    }

    public boolean leftTrigger() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.LeftTrigger > 0.5;
        }
    }

    public boolean left() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.Left;
        }
    }

    public double leftX() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.LeftX;
        }
    }

    public double leftY() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.LeftY;
        }
    }

    public boolean rightBumper() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.RightBumper;
        }
    }

    public boolean rightTrigger() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.RightTrigger > 0.5;
        }
    }

    public boolean right() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.Right;
        }
    }

    public double rightX() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.RightX;
        }
    }

    public double rightY() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.RightY;
        }
    }

    public boolean square() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.Square;
        }
    }

    public boolean triangle() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.Triangle;
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
            case IDLE: {
                yield State.IDLE;
            }

            case RUMBLING: {
                yield State.RUMBLING;
            }
        };
    }

    @Override
    protected void applyState() {
        switch (m_state) {
            case IDLE: {
                m_controllerIO.get().setRumble(0.0);
                break;
            }

            case RUMBLING: {
                m_controllerIO.get().setRumble(1.0);
                break;
            }
        }
    }

    @Override
    public void periodic() {
        synchronized (m_controllerInputs) {
            Logger.processInputs(m_name, m_controllerInputs);
            m_state = updateState();
            Logger.recordOutput(m_name + "/DesiredState", m_desiredState);
            Logger.recordOutput(m_name + "/State", m_state);
            applyState();
        }
    }
}
