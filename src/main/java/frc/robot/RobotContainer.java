package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.robot.subsystems.kitbot.KitbotIntake;
import frc.robot.subsystems.kitbot.KitbotLauncher;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {
    private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            OperatorConstants.kDriverControllerPort);
    
    private final SendableChooser<Command> autoChooser;
    public static ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
    private SwerveDrive m_driveBase = SwerveDrive.getInstance();
    private KitbotIntake m_intake;
    private KitbotLauncher m_launcher;

    // Temporary code for testing climb
    private TalonFX m_climbMotor;
    private double m_climbPosition;

    private static Alliance m_alliance = Alliance.Blue;

    public RobotContainer() {
        if (Constants.kRobotName.equals("kitbot")) {
                m_intake = KitbotIntake.getInstance();
                m_launcher = KitbotLauncher.getInstance();
        } else if (Constants.kRobotName.equals("hades")) {
                // Temporary code for testing climb
                m_climbMotor = new TalonFX(19);
                TalonFXConfiguration configs = new TalonFXConfiguration();
                configs.Slot0.kP = 1.0;
                m_climbMotor.getConfigurator().apply(configs);
                m_climbPosition = m_climbMotor.getPosition().getValueAsDouble();
        }

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
                        .onTrue(m_intake.intake())
                        .onTrue(m_launcher.intake())
                        .onFalse(m_intake.off())
                        .onFalse(m_launcher.off());
                
                // // launches fuel -- right bumper of driver
                // controller
                m_driverController
                        .R1()
                        .onTrue(m_intake.launch())
                        .onTrue(m_launcher.launch())
                        .onFalse(m_intake.off())
                        .onFalse(m_launcher.off());
        } else if (Constants.kRobotName.equals("hades")) {
                // Temporary code for testing climb
                System.out.println("Running Hades");
                m_driverController
                        .L1()
                        .whileTrue(new RunCommand(() -> {
                                m_climbPosition += 0.3;
                                System.out.println("Going up to " + m_climbPosition);
                                m_climbMotor.setControl(new PositionVoltage(m_climbPosition).withSlot(0));
                        }));

                m_driverController
                        .R1()
                        .whileTrue(new RunCommand(() -> {
                                m_climbPosition -= 0.3;
                                System.out.println("Going down to " + m_climbPosition);
                                m_climbMotor.setControl(new PositionVoltage(m_climbPosition).withSlot(0));
                        }));
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
