package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerve;
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
    private final CommandPS5Controller m_driverController = new CommandPS5Controller(
        OperatorConstants.getDriverControllerPort());
    private final CommandXboxController m_operatorController = new CommandXboxController(
        OperatorConstants.getOperatorControllerPort());
    
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
                .y()
                .onTrue(m_indexer.index().alongWith(m_launcher.launch()))
                .onFalse(m_indexer.off().alongWith(m_launcher.off()));
                
            // Increases launcher speed by 10% -- b button on operator controller
            m_operatorController
                .b()
                .onTrue(m_launcher.raiseSpeed());
            
            // Decreases launcher speed by 10% -- x button of operator controller
            m_operatorController
                .x()
                .onTrue(m_launcher.lowerSpeed());

            // Control the climb manually -- left and right bumpers of driver controller
            m_climb.setDefaultCommand(new RunCommand(() -> m_climb.manualControl(() -> {
                if (m_operatorController.leftBumper().getAsBoolean()) {
                    return 1.0;
                } else if (m_operatorController.rightBumper().getAsBoolean()) {
                    return -1.0;
                } else {
                    return 0.0;
                }
            }), m_climb));

            m_operatorController.leftBumper().and(() -> m_operatorController.rightBumper().getAsBoolean()).whileFalse(new RunCommand(() -> {
                m_climb.resetDesiredPosition();
            }, m_climb));

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
                .a()
                .onTrue(m_intakeDriver.intake())
                .onFalse(m_intakeDriver.off());
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static Alliance getAlliance() {
        return m_alliance;
    }

    public void periodic() {
      m_alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    private void createNamedCommands() {
        if (Constants.kRobotName.equals("tusk")) {
            NamedCommands.registerCommand("ClimbL1", m_climb.climbL1Height().andThen(m_climb.retract()));
            NamedCommands.registerCommand("StartIntake", m_intakePivot.out().alongWith(m_intakeDriver.intake()));
            NamedCommands.registerCommand("EndIntake", m_intakePivot.in().alongWith(m_intakeDriver.off()));
            NamedCommands.registerCommand("StartLaunch", m_indexer.index().alongWith(m_launcher.launch()));
            NamedCommands.registerCommand("EndLaunch", m_indexer.off().alongWith(m_launcher.off()));
        }
    }
}
