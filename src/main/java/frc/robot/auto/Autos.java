package frc.robot.auto;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.field.FieldDimensions;
import frc.robot.field.SideRelativePose2d;
import frc.robot.field.FieldDimensions.Side;

public final class Autos {
    private static final SideRelativePose2d TRENCH_START = new SideRelativePose2d(4.35, 0.6, Rotation2d.fromDegrees(-90.0));
    private static final SideRelativePose2d HUB_START = new SideRelativePose2d(3.6, FieldDimensions.FIELD_WIDTH.in(Meters) / 2.0, Rotation2d.fromDegrees(0.0));
    private static final SideRelativePose2d CLOSE_INTAKE_READY = new SideRelativePose2d(5.9, 1.2, Rotation2d.fromDegrees(-65.0));
    private static final SideRelativePose2d CLOSE_JUST_INTAKED = new SideRelativePose2d(5.9, 3.5, Rotation2d.fromDegrees(-65.0));
    private static final SideRelativePose2d CLOSE_RETURN_READY = new SideRelativePose2d(5.9, 0.6, Rotation2d.fromDegrees(180.0));
    private static final SideRelativePose2d FAR_INTAKE_READY = new SideRelativePose2d(7.7, 1.2, Rotation2d.fromDegrees(-65.0));
    private static final SideRelativePose2d FAR_JUST_INTAKED = new SideRelativePose2d(7.7, 3.5, Rotation2d.fromDegrees(-65.0));
    private static final SideRelativePose2d FAR_RETURN_READY = new SideRelativePose2d(7.7, 0.6, Rotation2d.fromDegrees(180.0));
    private static final SideRelativePose2d TRENCH_LAUNCH = new SideRelativePose2d(4.0, 0.64, Rotation2d.fromDegrees(83.25));

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        MetersPerSecond.of(4.7),
        MetersPerSecondPerSecond.of(4.7),
        DegreesPerSecond.of(270.0),
        DegreesPerSecondPerSecond.of(360.0),
        Volts.of(12.0)
    );

    public static Side side = Side.Outpost;
    private static final SendableChooser<Side> sideChooser = new SendableChooser<>();
    static {
        sideChooser.setDefaultOption("Outpost", Side.Outpost);
        sideChooser.addOption("Depot", Side.Depot);
        sideChooser.onChange((newSide) -> side = newSide);
    }

    private static Pose2d startingPosition = TRENCH_START;
    private static final SendableChooser<Pose2d> startChooser = new SendableChooser<>();
    static {
        startChooser.setDefaultOption("Trench", TRENCH_START);
        startChooser.addOption("Hub", HUB_START);
        startChooser.onChange((newStartingPosition) -> startingPosition = newStartingPosition);
    }

    private static Cycles cycles = Cycles.Zero;
    private static final SendableChooser<Cycles> cycleChooser = new SendableChooser<>();
    static {
        cycleChooser.setDefaultOption("None", Cycles.Zero);
        cycleChooser.addOption("One", Cycles.One);
        cycleChooser.addOption("Two", Cycles.Two);
        cycleChooser.onChange((newCycles) -> cycles = newCycles);
    }

    public static void addChoosersToShuffleboard(ShuffleboardTab tab) {
        tab.add("Side", sideChooser);
        tab.add("Starting Position", startChooser);
        tab.add("Cycles", cycleChooser);
    }

    public static Command getAuto(RobotContainer robot) {
        SequentialCommandGroup auto = new SequentialCommandGroup();

        auto.addCommands(
            robot.autoResetPose(startingPosition),

            switch (cycles) {
                case Zero -> Commands.none();
                case One -> farCycle(robot);
                case Two -> farCycle(robot).andThen(closeCycle(robot));
            }
        );

        return auto;
    }

    private static Command closeCycle(RobotContainer robot) {
        return goTo(CLOSE_INTAKE_READY, MetersPerSecond.of(2.5))
            .deadlineFor(robot.autoStartIntake())
            .andThen(goTo(CLOSE_JUST_INTAKED))
            .andThen(goTo(CLOSE_RETURN_READY)
                .andThen(goTo(TRENCH_LAUNCH))
                .deadlineFor(robot.autoEndIntake()))
            .andThen(Commands.waitSeconds(3.0).deadlineFor(robot.autoStartLaunch()));
    }

    private static Command farCycle(RobotContainer robot) {
        return goTo(FAR_INTAKE_READY, MetersPerSecond.of(2.5))
            .deadlineFor(robot.autoStartIntake())
            .andThen(goTo(FAR_JUST_INTAKED))
            .andThen(goTo(FAR_RETURN_READY)
                .andThen(goTo(TRENCH_LAUNCH))
                .deadlineFor(robot.autoEndIntake()))
            .andThen(Commands.waitSeconds(3.0).deadlineFor(robot.autoStartLaunch()));
    }

    private static Command goTo(Pose2d pose) {
        return goTo(pose, MetersPerSecond.of(0.0));
    }

    private static Command goTo(Pose2d pose, LinearVelocity endVelocity) {
        return AutoBuilder.pathfindToPose(pose, CONSTRAINTS, endVelocity);
    }

    private enum Cycles {
        Zero,
        One,
        Two,
    }

    private Autos() {}
}
