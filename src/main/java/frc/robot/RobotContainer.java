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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LockRotationTowardsHub;
import frc.robot.commands.TeleopSwerve;
import frc.robot.controllers.UniversalController;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.intake.IntakeDriver;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.kitbot.KitbotIntake;
import frc.robot.subsystems.kitbot.KitbotLauncher;
// import frc.robot.subsystems.launcher.Hood;
import frc.robot.subsystems.launcher.Indexer;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.swerve.SwerveDrive;

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
    // private Hood m_hood;
    private IntakeDriver m_intakeDriver;
    private IntakePivot m_intakePivot;
    private Climb m_climb;

    private static Alliance m_alliance = Alliance.Blue;
    private boolean m_limitOverrideMode = false;

    public RobotContainer() {
        if (Constants.kRobotName.equals("kitbot")) {
            m_kitbotIntake = KitbotIntake.getInstance();
            m_kitbotLauncher = KitbotLauncher.getInstance();
        } else if (Constants.kRobotName.equals("tusk")) {
            m_launcher = new Launcher();
            m_indexer = new Indexer();
            // m_hood = new Hood();
            m_climb = new Climb();
            m_intakeDriver = new IntakeDriver();
            m_intakePivot = new IntakePivot();
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

        // Slows speed and drives robot relative -- right trigger of driver controller
        m_driverController
            .R2()
            .whileTrue(
                new TeleopSwerve(
                    m_driverController,
                    OperatorConstants.kThrottleAxis,
                    OperatorConstants.kStrafeAxis,
                    OperatorConstants.kSteerAxis,
                    OperatorConstants.getSlowSpeed(),
                    true,
                    false
                )
            );

        // Make gyroscope think current position is zero -- create button of driver controller
        m_driverController
            .create()
            .onTrue(Commands.runOnce(() -> m_driveBase.zeroGyro(), m_driveBase));
        
        // Enables Palantir-Class Target Lock on the hub
        m_driverController
            .triangle()
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
            .onTrue(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 1)))
            .onFalse(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 0)));

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
                .onTrue(m_indexer.index().alongWith(m_launcher.launch()))
                .onFalse(m_indexer.off().alongWith(m_launcher.off()));
                
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
                if (m_operatorController.L1().getAsBoolean()) {
                    return 1.0;
                } else if (m_operatorController.R1().getAsBoolean()) {
                    return -1.0;
                } else {
                    return 0.0;
                }
            }, m_limitOverrideMode), m_climb));

            // Control the intake pivot manually -- left and right buttons on d-pad of operator controller
            m_intakePivot.setDefaultCommand(new RunCommand(() -> m_intakePivot.manualControl(() -> {
                if (m_operatorController.povRight().getAsBoolean()) {
                    return 1.0;
                } else if (m_operatorController.povLeft().getAsBoolean()) {
                    return -1.0;
                } else {
                    return 0.0;
                }
            }, m_limitOverrideMode), m_intakePivot));

            // Control the hood manually -- up and down arrows of operator controller
            // m_hood.setDefaultCommand(new RunCommand(() -> m_hood.manualControl(() -> {
            //     if (m_operatorController.povUp().getAsBoolean() ^ m_operatorController.povDown().getAsBoolean()) {
            //         return 0.0;
            //     } else if (m_operatorController.povUp().getAsBoolean()) {
            //         return 1.0;
            //     } else {
            //         return -1.0;
            //     }
            // }), m_hood));
            
            // Intakes fuel -- a button on operator controller
            m_operatorController
                .cross()
                .onTrue(m_intakeDriver.intake())
                .onFalse(m_intakeDriver.off());

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

    public static Alliance getAlliance() {
        return m_alliance;
    }

    public Alliance getAutoWinner() {
        switch (DriverStation.getGameSpecificMessage()) {
            case "R":
                return Alliance.Red;
            case "B":
                return Alliance.Blue;
            default:
                return null;
        }
    }

    public boolean aboutToSwitch() {
        double matchTime = DriverStation.getMatchTime();

        return DriverStation.isTeleop()
            && (
                (matchTime <= 131 && matchTime > 130)
                || (matchTime <= 106 && matchTime > 105)
                || (matchTime <= 81 && matchTime > 80)
                || (matchTime <= 56 && matchTime > 55)
                || (matchTime <= 31 && matchTime > 30)
                || (matchTime <= 1 && matchTime > 0)
            );
    }

    public void periodic() {
      m_alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
      SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    private void createNamedCommands() {
        if (Constants.kRobotName.equals("tusk")) {
            NamedCommands.registerCommand("ClimbPrepare", m_climb.climbL1Height());
            NamedCommands.registerCommand("Climb", m_climb.L1ClimbRetract());
            NamedCommands.registerCommand("StartIntake", m_intakePivot.out().alongWith(m_intakeDriver.intake()));
            NamedCommands.registerCommand("EndIntake", m_intakePivot.in().alongWith(m_intakeDriver.off()));
            NamedCommands.registerCommand("StartLaunch", m_indexer.index().alongWith(m_launcher.launch()));
            NamedCommands.registerCommand("EndLaunch", m_indexer.off().alongWith(m_launcher.off()));
        }
    }
}
