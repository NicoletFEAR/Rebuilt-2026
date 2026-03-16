package frc.robot.robots;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.robots.base.RobotContainer;
import frc.robot.robots.hades.Hades;
import frc.robot.robots.kitbot.Kitbot;
import frc.robot.robots.tusk.Tusk;
import frc.robot.robots.unrecognized.Unrecognized;

/**
 * Represents the robot running the code.
 */
public enum RobotIdentity {
    /**
     * Hades is just a drivebase, and it has its electrical on the bottom (hence the name).
     * The team number of its roboRIO should be 4785.
     */
    HADES("Hades"),
    /**
     * The Kitbot has its electrical on top of the drivebase, and is reused from year to year,
     * just with different mechs. The team number of its roboRIO should be 4784.
     */
    KITBOT("Kitbot"),
    /**
     * Tusk is our competition robot this year. The team number of its roboRIO should (of course)
     * be 4786.
     */
    TUSK("Tusk"),
    /**
     * In case we're connected to a roboRIO whose team number isn't one of 4784, 4785, or 4786.
     */
    UNRECOGNIZED("Unrecognized"),
    ;

    private final String m_name;

    RobotIdentity(String name) {
        m_name = name;
    }

    /**
     * Returns the name of the robot as a {@link java.lang.String String}.
     * 
     * @return The name of the robot, formatted with proper capitalization.
     */
    @Override
    public String toString() {
        return m_name;
    }

    /**
     * Gets the correct {@link frc.robot.robots.base.RobotContainer RobotContainer} for the
     * robot running.
     * 
     * @return The {@code RobotContainer} for the robot.
     */
    public RobotContainer getRobot() {
        return switch (this) {
            case HADES -> new Hades();
            case KITBOT -> new Kitbot();
            case TUSK -> new Tusk();
            case UNRECOGNIZED -> new Unrecognized();
        };
    }

    /**
     * Uses the team number of the connected roboRIO to determine which robot is running.
     * 
     * @return The running robot's identity.
     */
    public static RobotIdentity getIdentity() {
        return switch (RobotController.getTeamNumber()) {
            case 4784 -> KITBOT;
            case 4785 -> HADES;
            case 4786 -> TUSK;
            default -> UNRECOGNIZED;
        };
    }
}
