package frc.robot.constants;

/**
 * General constants for the entire codebase.
 */
public final class Constants {
    /**
     * If true, will run in replay mode based on the log file currently
     * open in AdvantageScope.
     */
    public static final boolean kIsReplay = false;
    /**
     * The distance from a point in the state space that is deemed close
     * enough for the two points to be considered equal. Taken as a
     * proportion of the range of the state space. For example, if the
     * space ranges from 0 to 0.5 rotations, then a value of 0.0001
     * here means that being off by 0.00005 rotations is acceptable.
     */
    public static final double kGeneralTolerance = 0.0001;

    private Constants() {}
}
