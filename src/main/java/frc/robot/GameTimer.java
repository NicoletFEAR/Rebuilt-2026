package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.led.Led;
import frc.robot.controllers.UniversalController;

/**
 * This class is used to report out shifts in the game so that the driver
 * and mech operator are aware of when scoring and defence should occur.
 */
public class GameTimer extends Timer {

    /**
     * The phase of the match that is going to occur along with
     * the amount of time that each phase will take.  In the
     * Teleop phase, there are four shifts between Offense
     * and Defence, and the last 30 seconds is considered Endgame.
     */
    public enum Phase {
        AUTO(20),
        TRANSITION(10),
        OFFENSE(25),
        DEFENSE(25),
        ENDGAME(30);

        private final int m_duration;
        private Phase(int duration) {
            m_duration = duration;
        }

        public int getDuration() {
            return m_duration;
        }
    }

    private Phase m_phase = Phase.TRANSITION;
    private boolean wonAuto = false;
    private UniversalController m_operatorController;
    private UniversalController m_driverController;
    private Led m_led;
    private int m_shiftCount = 0;

    // The max number of shifts in the game
    private static final int MAX_SHIFTS = 4;

    // The amount of time before the next phase that should have
    // some level of alert
    private static final int BUFFER_TIME = 5;

    /**
     * Default constructor
     */
    public GameTimer(UniversalController driverController, UniversalController operatorController, Led led) {
        super();
        this.m_driverController = driverController;
        this.m_operatorController = operatorController;
        this.m_led = led;
    }

    /**
     * The team that won auto starts on defence for the first
     * shift
     * @param wonAuto
     */
    public void setWonAuto(boolean wonAuto) {
        this.wonAuto = wonAuto;
    }

    /**
     * This mentod is called when totlop starts
     * @param wonAuto Did this alliance win auto
     */
    public void teleopStart(boolean wonAuto) {
        // Start the timer
        super.start();
        this.wonAuto = wonAuto;

        // Once teleop starts, there is a transition phase
        m_phase = Phase.TRANSITION;

        // Set the starting phase based on who won autos
        sendNotices(m_phase);
    }

    /**
     * The the current shft number
     * @return int, with a value between 0 (pre-shift) and 4
     */
    public int getShiftCount() {
        return m_shiftCount;
    }

    /**
     * Get the current phase of the match
     * @return Phase
     */
    public Phase getPhase() {
        return m_phase;
    }

    /**
     * This method is called periodically and will determine when to send alerts
     */
    public void periodic() {
        double now = get();
        // TODO Complete this implementation
    }

    /**
     * This method will determine if the current time is within the notification
     * window for the robot, driver, and operator
     * @param time The current time
     * @return true / false
     */
    public boolean inNotificationWindow(double time) {
        // TODO Complete this method implementation
        return false;
    }

    /**
     * This method will send out notices to the driver, operator, leds, and dashboard
     * @param phase The upcoming phase
     */
    private void sendNotices(Phase phase) {
        // TODO Complete this method implementation
    }
}
