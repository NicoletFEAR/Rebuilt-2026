package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.kitbot.KitbotIntake;
import frc.robot.subsystems.kitbot.KitbotLauncher;
import frc.robot.subsystems.launcher.Indexer;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {
    private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            OperatorConstants.kDriverControllerPort);
    
    private final SendableChooser<Command> autoChooser;
    public static ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");

    private final SwerveDrive m_driveBase = SwerveDrive.getInstance();

    private KitbotIntake m_kitbotIntake;
    private KitbotLauncher m_kitbotLauncher;
    private Launcher m_launcher;
    private Indexer m_indexer;

    private Climb m_climb;

    private static Alliance m_alliance = Alliance.Blue;

    public RobotContainer() {
        if (Constants.kRobotName.equals("kitbot")) {
                m_kitbotIntake = KitbotIntake.getInstance();
                m_kitbotLauncher = KitbotLauncher.getInstance();
        } else if (Constants.kRobotName.equals("hades")) {
                m_climb = new Climb();
        } else if (Constants.kRobotName.equals("tusk")) {
                m_launcher = new Launcher();
                m_indexer = new Indexer();
                m_climb = new Climb();
        }

        createNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();

        m_mainTab.add("Auto Chooser", autoChooser).withPosition(5, 0).withSize(5, 2);

        configureBindings();
    }

    private void configureBindings() {
        // Driver Controller Mappings \\

        // // Driving -- joysticks of driver controller
        m_driveBase.setDefaultCommand(
                new TeleopSwerve(
                        m_driverController,
                        OperatorConstants.kThrottleAxis,
                        OperatorConstants.kStrafeAxis,
                        OperatorConstants.kSteerAxis,
                        OperatorConstants.kDefaultSpeed,
                        true,
                        true
                ));

        // // Slows speed -- left trigger of driver controller
        m_driverController
                .L2()
                .whileTrue(
                        new TeleopSwerve(
                                m_driverController,
                                OperatorConstants.kThrottleAxis,
                                OperatorConstants.kStrafeAxis,
                                OperatorConstants.kSteerAxis,
                                OperatorConstants.kSlowSpeed,
                                true,
                                true
                        ));

        // // Slows speed and drives robot relative -- right trigger of driver
        // controller
        m_driverController
                .R2()
                .whileTrue(
                        new TeleopSwerve(
                                m_driverController,
                                OperatorConstants.kThrottleAxis,
                                OperatorConstants.kStrafeAxis,
                                OperatorConstants.kSteerAxis,
                                OperatorConstants.kSlowSpeed,
                                true,
                                false
                        ));

        // // Make gyroscope think current position is zero -- create button of driver
        // controller
        m_driverController
                .create()
                .onTrue(Commands.runOnce(() -> m_driveBase.zeroGyro(), m_driveBase));
        
        // These are the controls for the kitbot subsystems
        if (Constants.kRobotName.equals("kitbot")) {
                // // Intakes fuel -- left bumper of driver
                // controller
                m_driverController
                        .L1()
                        .onTrue(m_kitbotIntake.intake())
                        .onTrue(m_kitbotLauncher.intake())
                        .onFalse(m_kitbotIntake.off())
                        .onFalse(m_kitbotLauncher.off());
                
                // // launches fuel -- right bumper of driver
                // controller
                m_driverController
                        .R1()
                        .onTrue(m_kitbotIntake.launch())
                        .onTrue(m_kitbotLauncher.launch())
                        .onFalse(m_kitbotIntake.off())
                        .onFalse(m_kitbotLauncher.off());
        } else if (Constants.kRobotName.equals("hades")) {
                // Control the climb manually -- left and right bumpers of driver controller
                m_climb.setDefaultCommand(new RunCommand(() -> m_climb.manualControl(() -> {
                        if (m_driverController.L1().getAsBoolean() ^ m_driverController.R1().getAsBoolean()) {
                            return 0.0;
                        } else if (m_driverController.L1().getAsBoolean()) {
                            return 1.0;
                        } else {
                            return -1.0;
                        }
                }), m_climb));
        } else if (Constants.kRobotName.equals("tusk")) {
                m_driverController
                        .L1()
                        .onTrue(m_launcher.launch())
                        .onTrue(m_indexer.launch())
                        .onFalse(m_launcher.off())
                        .onFalse(m_indexer.off());
                
                m_climb.setDefaultCommand(new RunCommand(() -> m_climb.manualControl(() -> {
                        if (m_driverController.L1().getAsBoolean() ^ m_driverController.R1().getAsBoolean()) {
                            return 0.0;
                        } else if (m_driverController.L1().getAsBoolean()) {
                            return 1.0;
                        } else {
                            return -1.0;
                        }
                }), m_climb));
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
        // NamedCommands.registerCommand("ClimbL1", m_climb.climbL1Height().andThen(m_climb.retract()));
    }
}
