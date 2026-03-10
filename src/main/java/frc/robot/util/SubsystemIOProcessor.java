package frc.robot.util;

import frc.lib.architecture.SubsystemIO;
import frc.robot.Constants;

/**
 * Runs a {@link frc.lib.architecture.SubsystemIO SubsystemIO} object along with its
 * inputs object.
 * 
 * Intended to be run in a separate thread to ensure that if waiting for sensors takes
 * too long, it doesn't cause a loop overrun.
 * 
 * @param <T> the type of the inputs
 */
public class SubsystemIOProcessor<T> implements Runnable {
    private SubsystemIO<T> m_subsystemIO;
    private T m_subsystemInputs;
    private long m_timestamp;

    /**
     * Creates a new {@code SubsystemIOProcessor} with the specified
     * {@code SubsystemIO} object and inputs.
     * 
     * @param subsystemIO the {@code SubsystemIO} to
     * use
     * @param subsystemInputs the inputs object to use
     */
    public SubsystemIOProcessor(SubsystemIO<T> subsystemIO, T subsystemInputs) {
        m_subsystemIO = subsystemIO;
        m_subsystemInputs = subsystemInputs;
    }

    /**
     * Loops through
     * {@link frc.lib.architecture.SubsystemIO#refreshData() SubsystemIO.refreshData()}
     * and
     * {@link frc.lib.architecture.SubsystemIO#updateInputs(Object) SubsystemIO.updateInputs(Object)}.
     * 
     * Runs {@code SubsystemIO.updateInputs()} in a synchronized block to prevent
     * the data from being read by the subsystem during updates. Waits until 20
     * milliseconds have passed before repeating in order to synchronize with the rest
     * of the code.
     */
    public void run() {
        while (true) {
            m_timestamp = System.currentTimeMillis();
            m_subsystemIO.refreshData();

            synchronized (m_subsystemInputs) {
                m_subsystemIO.updateInputs(m_subsystemInputs);
            }

            try {
                long difference = System.currentTimeMillis() - m_timestamp;

                if (difference < (long) (Constants.kdt * 1000.0)) {
                    Thread.sleep((long) (Constants.kdt * 1000.0) - difference);
                }
            } catch (InterruptedException e) {}
        }
    }
}
