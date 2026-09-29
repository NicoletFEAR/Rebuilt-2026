package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.DeviceIds;
import frc.robot.commands.AutoTarget;
import frc.robot.commands.TeleopSwerve;
import frc.robot.controllers.UniversalController;
import frc.robot.subsystems.intake.IntakeDriver;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.kitbot.KitbotIntake;
import frc.robot.subsystems.kitbot.KitbotLauncher;
import frc.robot.subsystems.launcher.Hood;
import frc.robot.subsystems.launcher.Indexer;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * This class is used to encapsulate the robot code, including all hardware
 * subsystems.
 * In the case that new functionality should be added, it will likely need to
 * added
 * to this class as it is the central point of all subsystems.
 */
public class RobotContainer {
    private final UniversalController m_driverController = new UniversalController(
            OperatorConstants.getDriverControllerPort(), DriveConstants.getControllerType());
    private final UniversalController m_operatorController = new UniversalController(
            OperatorConstants.getOperatorControllerPort(), OperatorConstants.getControllerType());

    private final SendableChooser<Command> autoChooser;
    public static ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");

    private final SwerveDrive m_driveBase = new SwerveDrive();

    private KitbotIntake m_kitbotIntake;
    private KitbotLauncher m_kitbotLauncher;

    private Launcher m_launcher;
    private Indexer m_indexer;
    private Hood m_hood;
    private IntakeDriver m_intakeDriver;
    private IntakePivot m_intakePivot;
    private Led m_led;

    private static Alliance m_alliance = Alliance.Blue;
    private boolean m_limitOverrideMode = false;
    private boolean m_automaticLaunching = false;
    private boolean m_manualLaunching = false;
    private boolean m_manualIndexing = false;

    private final GameTimer m_gameTimer = new GameTimer(m_driverController, m_operatorController, m_led);

    public RobotContainer() {
        if (Constants.kRobotName.equals("kitbot")) {
            m_kitbotIntake = KitbotIntake.getInstance();
            m_kitbotLauncher = KitbotLauncher.getInstance();
        } else if (Constants.kRobotName.equals("tusk")) {
            m_launcher = new Launcher();
            m_indexer = new Indexer();
            m_hood = new Hood(m_driveBase);
            m_intakeDriver = new IntakeDriver();
            m_intakePivot = new IntakePivot();
            m_led = new Led(DeviceIds.getLedID());
        }

        createNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();

        m_mainTab.add("Auto Chooser", autoChooser).withPosition(5, 0).withSize(5, 2);
        SmartDashboard.putBoolean("Enable Launch on the Fly", false);

        SmartDashboard.putData(
                "Testing/Test Swerve Modules",
                testSwerves());

        configureBindings();
    }

    private void configureBindings() {
        // Driver Controller Mappings \\

        // Driving -- joysticks of driver controller
        m_driveBase.setDefaultCommand(
                new TeleopSwerve(
                        m_driverController,
                        OperatorConstants.kThrottleAxis,
                        OperatorConstants.kStrafeAxis,
                        OperatorConstants.kSteerAxis,
                        OperatorConstants.getDefaultSpeed(),
                        true,
                        true,
                        m_driveBase));

        // Make gyroscope think current position is zero -- create button of driver
        // controller
        m_driverController
                .create()
                .onTrue(new InstantCommand(m_driveBase::zeroGyro, m_driveBase));

        // Enables Palantir-Class Target Lock on the hub
        m_operatorController
                .L2()
                .whileTrue(
                        new AutoTarget(
                                m_driverController,
                                OperatorConstants.kThrottleAxis,
                                OperatorConstants.kStrafeAxis,
                                OperatorConstants.getDefaultSpeed(),
                                true,
                                m_driveBase,
                                m_hood,
                                m_launcher));

        new Trigger(this::aboutToSwitch)
                .onTrue(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 1.0))
                        .alongWith(
                                new InstantCommand(() -> m_operatorController.setRumble(RumbleType.kBothRumble, 1.0)))
                        .alongWith(m_led.startScoringSwitchAnimation()))
                .onFalse(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 0.0))
                        .alongWith(
                                new InstantCommand(() -> m_operatorController.setRumble(RumbleType.kBothRumble, 0.0)))
                        .alongWith(m_led.startSwerveAnimation()));

        // These are the controls for the kitbot subsystems
        if (Constants.kRobotName.equals("kitbot")) {
            // Intakes fuel -- left bumper of driver controller
            m_driverController
                    .L1()
                    .onTrue(m_kitbotIntake.intake())
                    .onTrue(m_kitbotLauncher.intake())
                    .onFalse(m_kitbotIntake.off())
                    .onFalse(m_kitbotLauncher.off());

            // launches fuel -- right bumper of driver controller
            m_driverController
                    .R1()
                    .onTrue(m_kitbotIntake.launch())
                    .onTrue(m_kitbotLauncher.launch())
                    .onFalse(m_kitbotIntake.off())
                    .onFalse(m_kitbotLauncher.off());

            // These are the button bindings which are specific to Tusk
        } else if (Constants.kRobotName.equals("tusk")) {
            // TODO: Fix jostle distance in case the intake is stuck on a ball

            // // Reverses the Launcher direction -- triangle button of driver controller
            // m_driverController
            // .triangle()
            // .whileTrue(m_launcher.off()
            // .andThen(Commands.waitUntil(m_launcher::isAtVelocity))
            // .andThen(m_launcher.reverse()))
            // .onFalse(m_launcher.idle());

            // Intakes fuel -- left trigger on operator controller
            m_driverController
                    .L2()
                    .onTrue(m_led.startIntakeAnimation()
                            .alongWith(m_intakePivot.out())
                            .alongWith(m_intakeDriver.intake()))
                    .onFalse(m_led.startSwerveAnimation()
                            .alongWith(m_intakePivot
                                    .in()
                                    .until(m_intakePivot::isStuckOnBall)
                                    .andThen(new InstantCommand(m_intakePivot::resetDesiredPosition)))
                            .alongWith(m_intakeDriver.off()));

            // Extakes the balls inside the hopper -- right bumper of driver controller
            m_driverController
                    .R1()
                    .onTrue(m_intakePivot
                            .out()
                            .andThen(m_intakeDriver.extake()
                                    .alongWith(m_indexer.extake())))
                    .onFalse(m_intakePivot
                            .in()
                            .alongWith(m_intakeDriver.off())
                            .alongWith(m_indexer.off()));

            // Launches fuel by spinning up the launcher and then indexing the fuel -- right
            // trigger of operator controller
            m_driverController
                    .R2()
                    .whileTrue(m_led
                            .startLaunchAnimation()
                            .alongWith(m_launcher.launch())
                            .alongWith(new InstantCommand(() -> m_automaticLaunching = true))
                            .andThen(new WaitUntilCommand(m_launcher::isAtVelocity))
                            .andThen(m_driveBase.xWheels()
                                    .alongWith(m_indexer.index())
                                    .alongWith(m_intakePivot.in()
                                            .andThen(m_intakePivot.out()))
                                    .repeatedly()))
                    .onFalse(m_led.startSwerveAnimation()
                            .alongWith(new ConditionalCommand(
                                    new InstantCommand(),
                                    m_indexer.off(),
                                    () -> m_manualIndexing))
                            .alongWith(new ConditionalCommand(
                                    new InstantCommand(),
                                    m_launcher.idle(),
                                    () -> m_manualLaunching))
                            .alongWith(new InstantCommand(() -> m_automaticLaunching = false))
                            .alongWith(m_intakePivot
                                    .in()
                                    .until(m_intakePivot::isStuckOnBall)
                                    .andThen(new InstantCommand(m_intakePivot::resetDesiredPosition))));

            // Increases launcher speed by 10% unless it's already at 100% -- circle button
            // on operator controller
            m_driverController
                    .povRight()
                    .onTrue(m_launcher.raiseTrueSpeed());

            // Decreases launcher speed by 10% unless it's at 0% -- square button of
            // operator controller
            m_driverController
                    .povLeft()
                    .onTrue(m_launcher.lowerTrueSpeed());

            // Good speed for shooting from the middle of the alliance zone generally --
            // cross button of operator controller
            m_driverController
                    .cross()
                    .onTrue(m_launcher.setSpeedModifier(LauncherConstants.kAutoAimSpeeds.get(2.7))
                            .alongWith(m_hood.runProfileToPosition(LauncherConstants.kAutoAimHoodPositions.get(2.7))))
                    .onFalse(m_hood.runProfileToPosition(LauncherConstants.getHoodMinPosition()));

            // Good speed for shooting from the trench -- triangle button of operator
            // controller
            m_driverController
                    .triangle()
                    .onTrue(m_launcher.setSpeedModifier(LauncherConstants.kAutoAimSpeeds.get(3.65))
                            .alongWith(m_hood.runProfileToPosition(LauncherConstants.kAutoAimHoodPositions.get(3.65))))
                    .onFalse(m_hood.runProfileToPosition(LauncherConstants.getHoodMinPosition()));

            // Control the intake pivot manually -- left and right buttons on d-pad of
            // operator controller
            m_intakePivot.setDefaultCommand(new RunCommand(() -> m_intakePivot.manualControl(() -> {
                if (m_operatorController.povRight().getAsBoolean() == m_operatorController.povLeft().getAsBoolean()) {
                    return 0.0;
                } else if (m_operatorController.povRight().getAsBoolean()) {
                    return 1.0;
                } else {
                    return -1.0;
                }
            }, m_limitOverrideMode), m_intakePivot));

            // Control the hood manually -- up and down arrows of operator controller
            m_hood.setDefaultCommand(new RunCommand(() -> m_hood.manualControl(() -> {
                if (m_driverController.povUp().getAsBoolean() == m_driverController.povDown().getAsBoolean()) {
                    return 0.0;
                } else if (m_driverController.povUp().getAsBoolean()) {
                    return 1.0;
                } else {
                    return -1.0;
                }
            }, m_limitOverrideMode), m_hood));

            // m_operatorController
            // .leftStick()
            // .whileTrue(m_launcher.rampVoltage());

            // Disables limits for manual mechanism control -- create button on operator
            // control
            m_operatorController
                    .create()
                    .onTrue(new InstantCommand(() -> m_limitOverrideMode = true))
                    .onFalse(new InstantCommand(() -> m_limitOverrideMode = false));

            // Only indexes -- Left bumper on operator controller
            m_operatorController
                    .L1()
                    .onTrue(new InstantCommand(() -> m_manualIndexing = true).andThen(m_indexer.index()))
                    // .onFalse(m_launcher.off());
                    .onFalse(new InstantCommand(() -> m_manualIndexing = false).andThen(new ConditionalCommand(
                            new InstantCommand(),
                            m_indexer.off(),
                            () -> m_automaticLaunching)));

            // Only launches -- Right bumper on operator controller
            m_operatorController
                    .R1()
                    .onTrue(new InstantCommand(() -> m_manualLaunching = true).andThen(m_launcher.launch()))
                    // .onFalse(m_launcher.off());
                    .onFalse(new InstantCommand(() -> m_manualLaunching = false).andThen(new ConditionalCommand(
                            new InstantCommand(),
                            m_launcher.idle(),
                            () -> m_automaticLaunching)));
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private Command testSwerves() {
        SequentialCommandGroup testCommand = new SequentialCommandGroup();

        for (double speed = 0.0; speed < 4.0; speed += 1.0) {
            for (double angle = 0.0; angle < 360.0; angle += 15.0) {
                testCommand.addCommands(
                        runSwervesToState(speed, angle),
                        Commands.waitSeconds(0.1));
            }
        }

        testCommand.addCommands(runSwervesToState(0.0, 0.0), Commands.waitSeconds(1.0));

        for (double speed = -1.0; speed > -4.0; speed -= 1.0) {
            for (double angle = 0.0; angle < 360.0; angle += 15.0) {
                testCommand.addCommands(
                        runSwervesToState(speed, angle),
                        Commands.waitSeconds(0.1));
            }
        }

        testCommand.addCommands(runSwervesToState(0.0, 0.0));

        return testCommand;
    }

    private Command runSwervesToState(double speed, double angle) {
        return Commands.runOnce(() -> {
            SwerveModuleState state = new SwerveModuleState(
                    MetersPerSecond.of(speed),
                    Rotation2d.fromDegrees(angle));

            SwerveModuleState[] states = new SwerveModuleState[] {
                    state,
                    state,
                    state,
                    state,
            };

            Logger.recordOutput("Swerve Desired States", states);
            m_driveBase.setModuleStates(states, true);
        });
    }

    public Command runAutoLedAnimation() {
        return m_led.startAutoAnimation();
    }

    public Command stopLedAnimation() {
        return m_led.stopAnimation();
    }

    public static Alliance getAlliance() {
        return m_alliance;
    }

    public Alliance getAutoWinner() {
        String data = DriverStation.getGameSpecificMessage();

        if (data.length() > 0) {
            switch (data.charAt(0)) {
                case 'R':
                    return Alliance.Red;
                case 'B':
                    return Alliance.Blue;
                default:
                    return null;
            }
        } else {
            return null;
        }
    }

    // TODO: Replace by the GameTimer class
    public boolean aboutToSwitch() {
        double matchTime = DriverStation.getMatchTime();

        return DriverStation.isTeleop()
                && DriverStation.isFMSAttached()
                && ((matchTime <= 131.0d && matchTime > 130.0d)
                        || (matchTime <= 81.0d && matchTime > 80.0d)
                        || (matchTime <= 56.0d && matchTime > 55.0d)
                        || (matchTime <= 31.0d && matchTime > 30.0d)
                        || (matchTime <= 1.0d && matchTime > 0.0d));
    }

    public void periodic() {
        m_alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    /**
     * This method should be called at the start of autonomous and perform
     * any necessary setup and processing for the autonomous period
     */
    public void autonomousInit() {
        m_alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public Command autonomousInitCommand() {
        return m_hood.runProfileToPosition(0.0d)
                .alongWith(m_launcher.idle());
    }

    public Command adjustLauncherSpeedToHub() {
        return m_launcher.adjustSpeedToHubDistance(m_driveBase::distanceToHub);
    }

    /**
     * This method should be called at the end of autonomous and perform
     * any necessary cleanup for the autonomous period
     */
    public void autonomousExit() {
        CommandScheduler.getInstance().schedule(
                m_launcher.stopAutoAim()
                        .andThen(m_launcher.idle())
                        .alongWith(m_hood.runProfileToPosition(0.0))
                        .alongWith(m_intakeDriver.off())
                        .alongWith(m_intakePivot.in()));
    }

    /**
     * Ths mehtod should be called at the start of teleop and performs
     * any necessary setup and processing for the period. This includes determining
     * who won autos, starting times, and sending alerts based on game shifts.
     */
    public void teleopInit() {
        m_alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Start the game timer
        m_gameTimer.teleopStart(getAutoWinner() == m_alliance);

        CommandScheduler.getInstance().schedule(
                m_led.startSwerveAnimation()
                        .alongWith(m_launcher.idle())
                        .alongWith(m_indexer.off())
                        .alongWith(m_intakeDriver.off())
                        .alongWith(m_intakePivot.in())
                        .alongWith(m_launcher.setSpeedModifier(1.0))
                        .alongWith(m_hood.runProfileToPosition(0.0)));
    }

    public void teleopPeriodic() {
        m_gameTimer.periodic();
    }

    /**
     * This method should be called at the end of teleop and perform
     * any necessary cleanup for the teleop period
     */
    public void teleopExit() {
        CommandScheduler.getInstance().schedule(m_hood.runProfileToPosition(0.0d));
    }

    private void createNamedCommands() {
        if (Constants.kRobotName.equals("tusk")) {
            NamedCommands.registerCommand(
                    "ResetIntake",
                    m_intakePivot
                            .in()
                            .alongWith(m_intakeDriver.off()));

            NamedCommands.registerCommand(
                    "StartIntake",
                    m_intakePivot
                            .out()
                            .alongWith(m_intakeDriver.intake()));

            NamedCommands.registerCommand(
                    "EndIntake",
                    m_intakePivot
                            .in()
                            .alongWith(m_intakeDriver.off()));

            NamedCommands.registerCommand(
                    "StartLaunch",
                    m_launcher
                            .launch()
                            .alongWith(m_hood.adjustToHubDistance())
                            .alongWith(new WaitUntilCommand(m_launcher::isAtVelocity)
                                    .andThen(m_indexer.index()
                                            .alongWith(m_intakeDriver.intake())
                                            .alongWith(m_intakePivot.hold()
                                                    .andThen(m_intakePivot.in())
                                                    .repeatedly()))));

            NamedCommands.registerCommand(
                    "EndLaunch",
                    m_indexer
                            .off()
                            .alongWith(m_launcher.idle())
                            .alongWith(m_launcher.setSpeedModifier(1.0))
                            .alongWith(m_hood.endAutoTarget()));

            NamedCommands.registerCommand(
                    "StartExtake",
                    m_intakePivot
                            .out()
                            .andThen(m_intakeDriver.extake()
                                    .alongWith(m_indexer.extake())));

            NamedCommands.registerCommand(
                    "EndExtake",
                    m_intakePivot
                            .in()
                            .alongWith(m_intakeDriver.off()
                                    .alongWith(m_indexer.off())));
        }
    }
}
