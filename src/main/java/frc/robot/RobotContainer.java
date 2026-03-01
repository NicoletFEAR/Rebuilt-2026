package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DeviceIds;
import frc.robot.commands.LockRotationTowardsHub;
import frc.robot.commands.TeleopSwerve;
import frc.robot.controllers.UniversalController;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbState;
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
 * This class is used to encapsulate the robot code, including all hardware subsystems.  
 * In the case that new functionality should be added, it will likely need to added
 * to this class as it is the central point of all subsystems.
 */
public class RobotContainer {
    private final UniversalController m_driverController = new UniversalController(
        OperatorConstants.getDriverControllerPort(), DriveConstants.getControllerType());
    private final UniversalController m_operatorController = new UniversalController(
        OperatorConstants.getOperatorControllerPort(), OperatorConstants.getControllerType());
    
    private final SendableChooser<Command> autoChooser;
    public static ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");

    private final SwerveDrive m_driveBase = SwerveDrive.getInstance();

    private KitbotIntake m_kitbotIntake;
    private KitbotLauncher m_kitbotLauncher;

    private Launcher m_launcher;
    private Indexer m_indexer;
    private Hood m_hood;
    private IntakeDriver m_intakeDriver;
    private IntakePivot m_intakePivot;
    private Climb m_climb;
    private Led m_led;

    private static Alliance m_alliance = Alliance.Blue;
    private boolean m_limitOverrideMode = false;

    // private GameTimer m_gameTimer;

    public RobotContainer() {
        if (Constants.kRobotName.equals("kitbot")) {
            m_kitbotIntake = KitbotIntake.getInstance();
            m_kitbotLauncher = KitbotLauncher.getInstance();
        } else if (Constants.kRobotName.equals("tusk")) {
            m_launcher = new Launcher();
            m_indexer = new Indexer();
            m_hood = new Hood();
            m_climb = new Climb();
            m_intakeDriver = new IntakeDriver();
            m_intakePivot = new IntakePivot();
            m_led = new Led(DeviceIds.getLedID());
        }

        createNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();

        m_mainTab.add("Auto Chooser", autoChooser).withPosition(5, 0).withSize(5, 2);

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
                true
            )
        );

        // Slows speed -- left trigger of driver controller
        m_driverController
            .L2()
            .whileTrue(
                new TeleopSwerve(
                    m_driverController,
                    OperatorConstants.kThrottleAxis,
                    OperatorConstants.kStrafeAxis,
                    OperatorConstants.kSteerAxis,
                    OperatorConstants.getSlowSpeed(),
                    true,
                    true
                )
            );

        // Make gyroscope think current position is zero -- create button of driver controller
        m_driverController
            .create()
            .onTrue(new InstantCommand(() -> m_driveBase.zeroGyro(), m_driveBase));
        
        // Enables Palantir-Class Target Lock on the hub
        m_driverController
            .R2()
            .whileTrue(
                new LockRotationTowardsHub(
                    m_driverController,
                    OperatorConstants.kThrottleAxis,
                    OperatorConstants.kStrafeAxis,
                    OperatorConstants.getDefaultSpeed(),
                    true,
                    true
                )
            );
        
        new Trigger(() -> aboutToSwitch())
            .onTrue(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 1))
            .alongWith(m_led.startScoringSwitchAnimation()))
            .onFalse(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 0))
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
        } else if (Constants.kRobotName.equals("tusk")) {
            // Launches fuel by spinning up the launcher and then indexing the fuel -- y button of operator controller
            m_operatorController
                .triangle()
                .onTrue(m_led.startLaunchAnimation().alongWith(m_indexer.index()).alongWith(m_launcher.launch()))
                .whileTrue(m_intakePivot
                    .jostleOut()
                    .andThen(m_intakePivot
                        .in()
                        .until(m_intakePivot::isStuckOnBall)
                        .andThen(new InstantCommand(m_intakePivot::resetDesiredPosition))))
                .onFalse(m_led.startSwerveAnimation().alongWith(m_indexer.off()).alongWith(m_launcher.off()));
                
            // Increases launcher speed by 10% unless it's already at 100%, in which cases it goes back down to 10& -- b button on operator controller
            m_operatorController
                .circle()
                .onTrue(m_launcher.raiseSpeed());
            
            // Decreases launcher speed by 10% unless it's at 0%, in which case it goes back up to 100% -- x button of operator controller
            m_operatorController
                .square()
                .onTrue(m_launcher.lowerSpeed());

            // Control the climb manually -- left and right bumpers of operator controller
            m_climb.setDefaultCommand(new RunCommand(() -> m_climb.manualControl(() -> {
                if (m_operatorController.L1().getAsBoolean() == m_operatorController.R1().getAsBoolean()) {
                    return 0.0;
                } else if (m_operatorController.L1().getAsBoolean()) {
                    return 1.0;
                } else {
                    return -1.0;
                }
            }, m_limitOverrideMode), m_climb));

            // Retracts the climb when teleop starts after climbing in auto
            new Trigger(() -> DriverStation.isTeleopEnabled() && m_climb.getState() == ClimbState.RETRACT_AUTO)
                .onTrue(m_climb.climbL1());

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
            m_hood.setDefaultCommand(new RunCommand(() -> m_hood.manualControl(() -> {
                if (m_operatorController.povUp().getAsBoolean() == m_operatorController.povDown().getAsBoolean()) {
                    return 0.0;
                } else if (m_operatorController.povUp().getAsBoolean()) {
                    return 1.0;
                } else {
                    return -1.0;
                }
            }, m_limitOverrideMode), m_hood));
            
            // Intakes fuel -- a button on operator controller
            m_operatorController
                .cross()
                .onTrue(m_led.startIntakeAnimation()
                    .alongWith(m_intakePivot.out())
                    .alongWith(m_intakeDriver.intake()))
                .onFalse(m_led.startSwerveAnimation()
                    .alongWith(m_intakePivot
                        .in()
                        .until(() -> m_intakePivot.isStuckOnBall())
                        .andThen(new InstantCommand(() -> m_intakePivot.resetDesiredPosition())))
                        .alongWith(m_intakeDriver.off()));

            // Disables limits for manual mechanism control -- create button on operator control
            m_operatorController
                .create()
                .onTrue(new InstantCommand(() -> m_limitOverrideMode = true))
                .onFalse(new InstantCommand(() -> m_limitOverrideMode = false));
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
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

    public void periodic() {
      m_alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
      SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    /**
     * This method should be called at the start of autonomous and perform 
     * any necessary setup and processing for the autonomous period
     */
    public void autonomousInit() {}

    /**
     * This method should be called at the end of autonomous and perform
     * any necessary cleanup for the autonomous period
     */
    public void autonomousExit() {}

    /**
     * Ths mehtod should be called at the start of teleop and performs 
     * any necessary setup and processing for the period.  This includes determining 
     * who won autos, starting times, and sending alerts based on game shifts.
     */
    public void teleopInit() {
        // Start the game timer
        // m_gameTimer.teleopStart(getAutoWinner() == m_alliance);
        CommandScheduler.getInstance().schedule(m_led.startSwerveAnimation());
    }

    public void teleopPeriodic() {
        // m_gameTimer.periodic();
    }

    /**
     * This method should be called at the end of teleop and perform
     * any necessary cleanup for the teleop period
     */
    public void teleopExit() {}

    private void createNamedCommands() {
        if (Constants.kRobotName.equals("tusk")) {
            NamedCommands.registerCommand("ClimbPrepare", m_climb.climbL1());
            NamedCommands.registerCommand("Climb", m_climb.retractAuto());
            NamedCommands.registerCommand("StartIntake", m_intakePivot.out().alongWith(m_intakeDriver.intake()));
            NamedCommands.registerCommand("EndIntake", m_intakePivot.in().alongWith(m_intakeDriver.off()));
            NamedCommands.registerCommand("StartLaunch", m_indexer.index().alongWith(m_launcher.launch()));
            NamedCommands.registerCommand("EndLaunch", m_indexer.off().alongWith(m_launcher.off()));
        }
    }
}
