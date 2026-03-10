package frc.robot.subsystems.timing;

import edu.wpi.first.wpilibj.Alert;
import frc.lib.architecture.StateSubsystem;

public class TimingSubsystem {
    private final String m_name;
    private TimingIO m_timingIO;
    private final TimingIOInputsAutoLogged m_timingIOInputs = new TimingIOInputsAutoLogged();
    private final Alert m_missingIO;
    private State m_state = State.PRE_MATCH;

    public static enum State {
        AUTONOMOUS,
        BLUE_ACTIVE,
        ENDGAME,
        POST_AUTONOMOUS,
        PRE_MATCH,
        RED_ACTIVE,
        TRANSITION,
    }
}
