package frc.lib.architecture;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents a subsystem associated with a finite state machine.
 * 
 * A finite state machine is a system that has a set of possible states, along with a
 * set of rules to transition between them. For example, a very simple arm might have
 * two states: {@code MOVING} and {@code IDLE}. A set of transition rules for this arm
 * could be that if its state is {@code IDLE} and its desired position changes, its
 * state changes to {@code MOVING}, and if its state is {@code MOVING} and it reaches
 * its desired position, its state changes to {@code IDLE}.
 * <p>
 * This framework reduces unexpected subsystem behavior by clearly delineating every
 * possible scenario, makes debugging easier by breaking code down into more manageable
 * "chunks", one for each state, and makes future code modification easier and less
 * disruptive.
 * <p>
 * When implementing a {@code StateSubsystem}, it is best to have one field that keeps
 * track of the desired state, typically called {@code m_desiredState}, and one field
 * that keeps track of the actual system state, typically called {@code m_state},
 * since the two are not always the same. For example, if the desired state is
 * launching, the actual state could be launching, but it could also be spinning up
 * the launcher or waiting to be within range of the target. Thus, it is best to
 * implement two nested enums: one for the desired state, typically called
 * {@code DesiredState}, and one for the system state, typically called {@code State}.
 * 
 * @param <T> the underlying enum that represents the actual system state, not the
 * desired state
 */
public abstract class StateSubsystem<T> extends SubsystemBase {
    /**
     * Chooses the system state based on the desired state.
     * 
     * Takes into account the desired state along with other information like motor
     * currents or beam break values to determine the system state that should be
     * applied. Usually implemented with a switch statement on the desired state.
     * Should be called in the periodic method before {@link #applyState()}.
     * 
     * @return the system state to be applied
     */
    protected abstract T updateState();

    /**
     * Performs subsystem functions based on the system state.
     * 
     * Determines which actions to perform based on the system state and then executes
     * them. Usually implemented with a switch statement on the system state.
     * Should be called in the periodic method after {@link #updateState()}.
     */
    protected abstract void applyState();
}
