package frc.robot.util;

import java.util.ArrayList;

import frc.robot.Constants;

public class IOProcessor implements Runnable {
    private ArrayList<SubsystemIOProcessor<?>> m_processors = new ArrayList<SubsystemIOProcessor<?>>();
    private long m_timestamp = System.currentTimeMillis();

    public void add(SubsystemIOProcessor<?> processor) {
        m_processors.add(processor);
    }

    public void run() {
        while (true) {
            m_timestamp = System.currentTimeMillis();
            
            for (SubsystemIOProcessor<?> processor : m_processors) {
                processor.processIO();
            }

            try {
                long difference = System.currentTimeMillis() - m_timestamp;

                if (difference < (long) (Constants.kLoopTime * 1000.0)) {
                    Thread.sleep((long) (Constants.kLoopTime * 1000.0) - difference);
                }
            } catch (InterruptedException e) {}
        }
    }
}
