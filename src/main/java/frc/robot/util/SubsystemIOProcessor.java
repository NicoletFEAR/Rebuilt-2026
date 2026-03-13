package frc.robot.util;

import frc.lib.architecture.SubsystemIO;

/**
 * Runs a {@link frc.lib.architecture.SubsystemIO SubsystemIO} object along with its
 * inputs object.
 * 
 * Intended to be run by an {@link frc.robot.util.IOProcessor IOProcessor} in a separate thread
 * to ensure that if waiting for sensors takes too long, it doesn't cause a loop overrun.
 * 
 * @param <T> the type of the inputs
 */
public class SubsystemIOProcessor<T> {
    private Container<? extends SubsystemIO<T>> m_io;
    private T m_inputs;

    /**
     * Creates a new {@code SubsystemIOProcessor} with the specified
     * {@code SubsystemIO} object and inputs.
     * 
     * @param io the {@code SubsystemIO} to use, passed as a {@link frc.robot.util.Container Container}
     * to allow dynamic implementation changing
     *
     * @param inputs the inputs object to use
     */
    public SubsystemIOProcessor(Container<? extends SubsystemIO<T>> io, T inputs) {
        m_io = io;
        m_inputs = inputs;
    }

    /**
     * Runs through
     * {@link frc.lib.architecture.SubsystemIO#refreshData() SubsystemIO.refreshData()}
     * and
     * {@link frc.lib.architecture.SubsystemIO#updateInputs(Object) SubsystemIO.updateInputs(Object)}.
     * 
     * Runs {@code SubsystemIO.updateInputs()} in a synchronized block to prevent
     * the data from being read by the subsystem during updates. Intended to be run
     * by an {@code IOProcessor}.
     */
    public void processIO() {
        synchronized (m_io) {
            m_io.get().refreshData();

            synchronized (m_inputs) {
                m_io.get().updateInputs(m_inputs);
            }
        }
    }
}
