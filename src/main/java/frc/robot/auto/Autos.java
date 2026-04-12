package frc.robot.auto;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public final class Autos {
    private static final Pose2d TRENCH_START = new Pose2d(4.35, 0.6, Rotation2d.fromDegrees(-90.0));
    private static final Pose2d CLOSE_INTAKE_READY = new Pose2d(5.9, 1.2, Rotation2d.fromDegrees(-65.0));
    private static final Pose2d CLOSE_JUST_INTAKED = new Pose2d(5.9, 3.5, Rotation2d.fromDegrees(-65.0));
    private static final Pose2d CLOSE_RETURN_READY = new Pose2d(5.9, 0.6, Rotation2d.fromDegrees(180.0));
    private static final Pose2d FAR_INTAKE_READY = new Pose2d(7.7, 1.2, Rotation2d.fromDegrees(-65.0));
    private static final Pose2d FAR_JUST_INTAKED = new Pose2d(7.7, 3.5, Rotation2d.fromDegrees(-65.0));
    private static final Pose2d FAR_RETURN_READY = new Pose2d(7.7, 0.6, Rotation2d.fromDegrees(180.0));
    private static final Pose2d TRENCH_LAUNCH = new Pose2d(4.0, 0.64, Rotation2d.fromDegrees(83.25));

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        MetersPerSecond.of(4.7),
        MetersPerSecondPerSecond.of(4.7),
        DegreesPerSecond.of(270.0),
        DegreesPerSecondPerSecond.of(360.0),
        Volts.of(12.0)
    );

    public static SendableChooser<Supplier<Command>> buildAutoChooser(RobotContainer robot) {
        SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();
        chooser.setDefaultOption("None", () -> Commands.none());
        chooser.addOption("1-Cycle", () -> robot.autoResetPose(TRENCH_START).andThen(farCycle(robot)));
        chooser.addOption("2-Cycle", () -> robot.autoResetPose(TRENCH_START).andThen(twoCycle(robot)));
        return chooser;
    }

    public static Command twoCycle(RobotContainer robot) {
        return farCycle(robot).andThen(closeCycle(robot));
    }

    public static Command closeCycle(RobotContainer robot) {
        return goTo(CLOSE_INTAKE_READY, MetersPerSecond.of(2.5))
            .deadlineFor(robot.autoStartIntake())
            .andThen(goTo(CLOSE_JUST_INTAKED))
            .andThen(goTo(CLOSE_RETURN_READY)
                .andThen(goTo(TRENCH_LAUNCH))
                .deadlineFor(robot.autoEndIntake()))
            .andThen(Commands.waitSeconds(3.0).deadlineFor(robot.autoStartLaunch()));
    }

    public static Command farCycle(RobotContainer robot) {
        return goTo(FAR_INTAKE_READY, MetersPerSecond.of(2.5))
            .deadlineFor(robot.autoStartIntake())
            .andThen(goTo(FAR_JUST_INTAKED))
            .andThen(goTo(FAR_RETURN_READY)
                .andThen(goTo(TRENCH_LAUNCH))
                .deadlineFor(robot.autoEndIntake()))
            .andThen(Commands.waitSeconds(3.0).deadlineFor(robot.autoStartLaunch()));
    }

    public static Command followPath(String name) {
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(name));
        } catch (Exception e) {
            DriverStation.reportError("Couldn't load path '" + name + "'': " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    private static Command goTo(Pose2d pose) {
        return goTo(pose, MetersPerSecond.of(0.0));
    }

    private static Command goTo(Pose2d pose, LinearVelocity endVelocity) {
        return AutoBuilder.pathfindToPose(pose, CONSTRAINTS, endVelocity);
    }

    private Autos() {}
}
