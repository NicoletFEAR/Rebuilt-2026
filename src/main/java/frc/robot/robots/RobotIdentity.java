package frc.robot.robots;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.robots.base.RobotContainer;
import frc.robot.robots.tusk.Tusk;
import frc.robot.robots.unrecognized.Unrecognized;

public enum RobotIdentity {
    TUSK("Tusk"),
    UNRECOGNIZED("Unrecognized"),
    ;

    private final String name;

    RobotIdentity(String name) {
        this.name = name;
    }

    @Override
    public String toString() {
        return name;
    }

    public RobotContainer getRobot() {
        return switch (this) {
            case TUSK -> new Tusk();
            case UNRECOGNIZED -> new Unrecognized();
        };
    }

    public static RobotIdentity getIdentity() {
        return switch (RobotController.getTeamNumber()) {
            case 4786 -> TUSK;
            default -> UNRECOGNIZED;
        };
    }
}
