package frc.robot.util;

import frc.lib.architecture.SubsystemIO;
import frc.robot.Constants;

public class SubsystemIOProcessor<T> implements Runnable {
    private SubsystemIO<T> m_subsystemIO;
    private T m_subsystemInputs;
    private long m_timestamp;

    public SubsystemIOProcessor(SubsystemIO<T> subsystemIO, T subsystemInputs) {
        m_subsystemIO = subsystemIO;
        m_subsystemInputs = subsystemInputs;
    }

    public void run() {
        while (true) {
            m_timestamp = System.currentTimeMillis();
            m_subsystemIO.refreshInputs();

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
