package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.kitbot.IntakeAndLauncher;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {
    private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            OperatorConstants.kDriverControllerPort);
    
    private final SendableChooser<Command> autoChooser;
    public static ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
    private SwerveDrive m_driveBase = SwerveDrive.getInstance();
    private IntakeAndLauncher m_intakeAndLauncher = IntakeAndLauncher.getInstance();
    private static Alliance m_alliance = Alliance.Blue;

    public RobotContainer() {
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
        if (Constants.kRobotName == "kitbot") {
                // // Intakes fuel -- left bumper of driver
                // controller
                m_driverController
                        .L1()
                        .onTrue(m_intakeAndLauncher.intake())
                        .onFalse(m_intakeAndLauncher.off());
                
                // // launches fuel -- right bumper of driver
                // controller
                m_driverController
                        .R1()
                        .onTrue(m_intakeAndLauncher.launch())
                        .onFalse(m_intakeAndLauncher.off());
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
}
