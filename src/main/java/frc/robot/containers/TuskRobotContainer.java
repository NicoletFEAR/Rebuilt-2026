package frc.robot.containers;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GameTimer;
import frc.robot.commands.LockRotationTowardsHub;
import frc.robot.commands.TeleopSwerve;
import frc.robot.constants.GeneralConstants.GeneralOperatorConstants;
import frc.robot.constants.KitbotConstants.KitbotOperatorConstants;
import frc.robot.constants.TuskConstants.TuskDeviceIds;
import frc.robot.constants.TuskConstants.TuskOperatorConstants;
import frc.robot.controllers.UniversalController;
import frc.robot.subsystems.intake.IntakeDriver;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.launcher.Hood;
import frc.robot.subsystems.launcher.Indexer;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * This class is used to encapsulate the robot code, including all hardware subsystems.  
 * In the case that new functionality should be added, it will likely need to added
 * to this class as it is the central point of all subsystems.
 */
public class TuskRobotContainer extends AbstractRobotContainer {
    
    protected final UniversalController m_driverController = new UniversalController(
        GeneralOperatorConstants.kDriverControllerPort, TuskOperatorConstants.kDriveControllerType);
    protected final UniversalController m_operatorController = new UniversalController(
        GeneralOperatorConstants.kOperatorControllerPort, TuskOperatorConstants.kOperatorControllerType);
    
    private Launcher m_launcher;
    private Indexer m_indexer;
    private Hood m_hood;
    private IntakeDriver m_intakeDriver;
    private IntakePivot m_intakePivot;
    private Led m_led;
    private boolean m_limitOverrideMode = false;
    private boolean m_automaticLaunching = false;
    private boolean m_manualLaunching = false;
    private boolean m_manualIndexing = false;

    private final GameTimer m_gameTimer = new GameTimer(m_driverController, m_operatorController, m_led);

    public TuskRobotContainer(Alliance alliance) {
        super(BotEnum.TUSK, alliance);

        configureSwerveBindings();
    }

    @Override
    protected void createDriveBase() {
        m_driveBase = new SwerveDrive();
    }

    @Override
    protected void createRobotSubsystems() {
        this.m_launcher = new Launcher();
        this.m_indexer = new Indexer();
        this.m_hood = new Hood();
        this.m_intakeDriver = new IntakeDriver();
        this.m_intakePivot = new IntakePivot();
        this.m_led = new Led(TuskDeviceIds.kLEDId);
    }

    /**
    * This method is used to set the default bindings for the swerve drive.  This should
    * only be overridden in rare cases.  This is only called once per robot type.
    */
    private void configureSwerveBindings() {
        // Driving -- joysticks of driver controller
        m_driveBase.setDefaultCommand(
            new TeleopSwerve(
                m_driverController,
                KitbotOperatorConstants.kThrottleAxis,
                KitbotOperatorConstants.kStrafeAxis,
                KitbotOperatorConstants.kSteerAxis,
                GeneralOperatorConstants.kDefaultSpeed,
                true,
                true,
                m_driveBase
            )
        );

        // Slows speed -- left trigger of driver controller
        m_driverController
            .L2()
            .whileTrue(
                new TeleopSwerve(
                    m_driverController,
                    KitbotOperatorConstants.kThrottleAxis,
                    KitbotOperatorConstants.kStrafeAxis,
                    KitbotOperatorConstants.kSteerAxis,
                    GeneralOperatorConstants.kSlowSpeed,
                    true,
                    true,
                    m_driveBase
                )
            );
        
        // Make gyroscope think current position is zero -- create button of driver controller
        m_driverController
            .create()
            .onTrue(new InstantCommand(m_driveBase::zeroGyro, m_driveBase));
    }

    @Override
    protected void configureBindings() {
        
        // Enables Palantir-Class Target Lock on the hub
        m_driverController
            .R2()
            .whileTrue(
                new LockRotationTowardsHub(
                    m_driverController,
                    TuskOperatorConstants.kThrottleAxis,
                    TuskOperatorConstants.kStrafeAxis,
                    GeneralOperatorConstants.kDefaultSpeed,
                    true,
                    true,
                    m_driveBase
                )
            );
        
        new Trigger(this::aboutToSwitch)
            .onTrue(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 1.0))
            .alongWith(m_led.startScoringSwitchAnimation()))
            .onFalse(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 0.0))
            .alongWith(m_led.startSwerveAnimation()));

        
        // Adjusts hood and speed during auto-targeting -- right trigger of driver controller
        // m_driverController
        //     .R2()
        //     .whileTrue(m_launcher.adjustSpeedToHubDistance(m_driveBase::distanceToHub)
        //         .alongWith(m_hood.adjustToHubDistance(m_driveBase::distanceToHub)))
        //     .onFalse(m_hood.endAutoTarget());

        // TODO: Fix jostle distance in case the intake is stuck on a ball
        // Launches fuel by spinning up the launcher and then indexing the fuel -- right trigger of operator controller
        m_operatorController
            .R2()
            .whileTrue(m_led
                .startLaunchAnimation()
                .alongWith(m_launcher.launch())
                .alongWith(new InstantCommand(() -> m_automaticLaunching = true))
                .andThen(new WaitUntilCommand(m_launcher::isAtVelocity))
                .andThen(m_driveBase.xWheels()
                    .alongWith(m_indexer.index())
                    .alongWith(m_intakePivot.hold().andThen(m_intakePivot.in()).repeatedly())
                )
            ).onFalse(m_led.startSwerveAnimation()
                .alongWith(new ConditionalCommand(
                    new InstantCommand(),
                    m_indexer.off(),
                    () -> m_manualIndexing
                ))
                // .alongWith(m_indexer.off())
                .alongWith(new ConditionalCommand(
                    new InstantCommand(),
                    m_launcher.off(),
                    () -> m_manualLaunching
                ))
                // .alongWith(m_launcher.off())
                .alongWith(new InstantCommand(() -> m_automaticLaunching = false))
                .alongWith(m_intakePivot
                    .in()
                    .until(m_intakePivot::isStuckOnBall)
                    .andThen(new InstantCommand(m_intakePivot::resetDesiredPosition))
                ));
            
        // Increases launcher speed by 10% unless it's already at 100% -- circle button on operator controller
        m_operatorController
            .circle()
            .onTrue(m_launcher.raiseSpeed());
        
        // Decreases launcher speed by 10% unless it's at 0% -- square button of operator controller
        m_operatorController
            .square()
            .onTrue(m_launcher.lowerSpeed());
        
        // Good speed for shooting from the middle of the alliance zone generally -- cross button of operator controller
        m_operatorController
            .cross()
            .onTrue(m_launcher.setSpeedModifier(0.7));
        
        // Good speed for shooting from the trench -- triangle button of operator controller
        m_operatorController
            .triangle()
            .onTrue(m_launcher.setSpeedModifier(1.0));

        // Control the climb manually -- left and right bumpers of operator controller
        // m_climb.setDefaultCommand(new RunCommand(() -> m_climb.manualControl(() -> {
        //     if (m_operatorController.L1().getAsBoolean() == m_operatorController.R1().getAsBoolean()) {
        //         return 0.0;
        //     } else if (m_operatorController.L1().getAsBoolean()) {
        //         return 1.0;
        //     } else {
        //         return -1.0;
        //     }
        // }, m_limitOverrideMode), m_climb));

        // Retracts the climb when teleop starts after climbing in auto
        // new Trigger(() -> DriverStation.isTeleopEnabled() && m_climb.getState() == ClimbState.RETRACT_AUTO)
        //     .onTrue(m_climb.climbL1());

        // Control the intake pivot manually -- left and right buttons on d-pad of operator controller
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
        // m_hood.setDefaultCommand(new RunCommand(() -> m_hood.manualControl(() -> {
        //     if (m_operatorController.povUp().getAsBoolean() == m_operatorController.povDown().getAsBoolean()) {
        //         return 0.0;
        //     } else if (m_operatorController.povUp().getAsBoolean()) {
        //         return 1.0;
        //     } else {
        //         return -1.0;
        //     }
        // }, m_limitOverrideMode), m_hood));
        
        // Intakes fuel -- left trigger on operator controller
        m_operatorController
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

        // Disables limits for manual mechanism control -- create button on operator control
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
                () -> m_automaticLaunching
            )));
        
        // Only launches -- Right bumper on operator controller
        m_operatorController
            .R1()
            .onTrue(new InstantCommand(() -> m_manualLaunching = true).andThen(m_launcher.launch()))
            // .onFalse(m_launcher.off());
            .onFalse(new InstantCommand(() -> m_manualLaunching = false).andThen(new ConditionalCommand(
                new InstantCommand(),
                m_launcher.off(),
                () -> m_automaticLaunching
            )));
    }

    @Override
    protected void createNamedCommands() {
        // NamedCommands.registerCommand("ClimbPrepare", m_climb.climbL1());
        // NamedCommands.registerCommand("Climb", m_climb.retractAuto());
        // TODO: Replace these old commands with their newer versions in the autos
        // NamedCommands.registerCommand("HoodDown", new RunCommand(() -> m_hood.runToPosition(0.0)).until(m_hood::getIsAtSetpoint));
        NamedCommands.registerCommand("StartIntake", m_intakePivot.out().alongWith(m_intakeDriver.intake()));
        
        NamedCommands.registerCommand(
            "EndIntake",
            m_intakePivot.in().alongWith(m_intakeDriver.off())
        );

        NamedCommands.registerCommand(
            "StartLaunch",
            m_launcher
                .launch()
                .andThen(new WaitUntilCommand(m_launcher::isAtVelocity))
                .andThen(m_indexer.index())
        );

        NamedCommands.registerCommand(
            "StartLaunchSlow",
            m_launcher
                .setSpeedModifier(0.7)
                .andThen(m_launcher.launch())
                .andThen(new WaitUntilCommand(m_launcher::isAtVelocity))
                .andThen(m_indexer.index())
        );

        NamedCommands.registerCommand(
            "EndLaunch",
            m_indexer
                .off()
                .alongWith(m_launcher.off())
                .alongWith(m_launcher.setSpeedModifier(1.0))
        );
    }

    public Command runAutoLedAnimation() {
        return m_led.startAutoAnimation();
    }

    public Command stopLedAnimation() {
        return m_led.stopAnimation();
    }

    public Alliance getAutoWinner() {
        String data = DriverStation.getGameSpecificMessage();

        if (data.length() > 0){
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
            && (
                (matchTime <= 131.0d && matchTime > 130.0d)
                || (matchTime <= 81.0d && matchTime > 80.0d)
                || (matchTime <= 56.0d && matchTime > 55.0d)
                || (matchTime <= 31.0d && matchTime > 30.0d)
                || (matchTime <= 1.0d && matchTime > 0.0d)
            );
    }

    /**
     * Ths mehtod should be called at the start of teleop and performs 
     * any necessary setup and processing for the period.  This includes determining 
     * who won autos, starting times, and sending alerts based on game shifts.
     */
    @Override
    public void teleopInit() {
        super.teleopInit();
        CommandScheduler.getInstance().schedule(m_led.startSwerveAnimation());

        // Start the game timer
        m_gameTimer.teleopStart(getAutoWinner() == m_alliance);

        CommandScheduler.getInstance().schedule(
            m_led.startSwerveAnimation()
                .alongWith(m_launcher.off())
                .alongWith(m_indexer.off())
                .alongWith(m_intakeDriver.off())
                .alongWith(m_intakePivot.in())
                .alongWith(m_launcher.setSpeedModifier(1.0))
                // .alongWith(m_hood.runProfileToPosition(LauncherConstants.getHoodMaxPosition() * 0.80))
        );
    }

    @Override
    public void autonomousInit() {
        Command autonomousCommand = getAutonomousCommand();
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(runAutoLedAnimation().alongWith(autonomousCommand));
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        CommandScheduler.getInstance().schedule(stopLedAnimation());
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
