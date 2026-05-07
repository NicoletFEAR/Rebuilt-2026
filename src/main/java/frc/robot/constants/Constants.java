package frc.robot.constants;

/**
 * General constants for the entire codebase.
 */
public final class Constants {
    /**
     * If true, will run in replay mode based on the log file currently
     * open in AdvantageScope.
     */
    public static final boolean IS_REPLAY = false;
    /**
     * The distance from a point in the state space that is deemed close
     * enough for the two points to be considered equal. Taken as a
     * proportion of the range of the state space. For example, if the
     * space ranges from 0 to 0.5 rotations, then a value of 0.000001
     * here means that being off by 0.0000005 rotations is acceptable.
     */
    public static final double GENERAL_TOLERANCE = 0.000001;
    /**
     * The distance from a point in an unbounded state space that is
     * deemed close enough for the two points to be considered equal.
     * For example, this could be used for a motor that is allowed
     * to turn infinitely. This is just a flat number and shouldn't
     * be multiplied by anything.
     */
    public static final double UNBOUNDED_TOLERANCE = 0.0001;

    public static final double LOOP_PERIOD = 0.02;

    private Constants() {}
}
