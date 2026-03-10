package frc.robot.subsystems.controller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.architecture.StateSubsystem;
import frc.robot.subsystems.controller.ControllerIO.ControllerIOInputs;
import frc.robot.subsystems.controllers.ControllerIOInputsAutoLogged;
import frc.robot.util.SubsystemIOProcessor;

public class ControllerSubsystem extends StateSubsystem<ControllerSubsystem.State> {
    private final String m_name;
    private ControllerIO m_controllerIO;
    private ControllerIOInputsAutoLogged m_controllerInputs = new ControllerIOInputsAutoLogged();
    private DesiredState m_desiredState = DesiredState.IDLE;
    private State m_state = State.IDLE;

    public ControllerSubsystem(String name, ControllerIO controllerIO) {
        m_name = name;
        m_controllerIO = controllerIO;
        new Thread(new SubsystemIOProcessor<ControllerIOInputs>(m_controllerIO, m_controllerInputs)).start();
    }

    public static enum DesiredState {
        IDLE,
        RUMBLING,
    }

    public static enum State {
        IDLE,
        RUMBLING,
    }

    public double leftXValue() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.LeftX;
        }
    }

    public double leftYValue() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.LeftY;
        }
    }

    public double rightXValue() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.RightX;
        }
    }

    public double rightYValue() {
        synchronized (m_controllerInputs) {
            return m_controllerInputs.RightY;
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
                m_controllerIO.setRumble(0.0);
                break;
            }

            case RUMBLING: {
                m_controllerIO.setRumble(1.0);
                break;
            }
        }
    }

    @Override
    public void periodic() {
        synchronized (m_controllerInputs) {
            Logger.processInputs(m_name, m_controllerInputs);
            m_state = updateState();
            Logger.recordOutput(m_name + "/Desired State", m_desiredState);
            Logger.recordOutput(m_name + "/State", m_state);
            applyState();
        }
    }
}
