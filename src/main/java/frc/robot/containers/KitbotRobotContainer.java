package frc.robot.containers;

import frc.robot.commands.TeleopSwerve;
import frc.robot.constants.GeneralConstants.GeneralOperatorConstants;
import frc.robot.constants.HadesConstants.HadesDriveConstants;
import frc.robot.constants.HadesConstants.HadesOperatorConstants;
import frc.robot.constants.KitbotConstants.KitbotOperatorConstants;
import frc.robot.controllers.UniversalController;
import frc.robot.subsystems.kitbot.KitbotIntake;
import frc.robot.subsystems.kitbot.KitbotLauncher;
import frc.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class KitbotRobotContainer extends AbstractRobotContainer {

    private KitbotIntake m_kitbotIntake;
    private KitbotLauncher m_kitbotLauncher;

    public KitbotRobotContainer(Alliance alliance, HadesDriveConstants constants) {
       super(BotEnum.KITBOT, alliance, constants);
    }

    @Override
    protected void createDriveBase() {
        m_driveBase = new SwerveDrive(m_constants);
    }

    @Override
    protected void createContollers() {
        this.m_driverController = new UniversalController(GeneralOperatorConstants.kDriverControllerPort, HadesOperatorConstants.kDriveControllerType);
    }

    @Override
    protected void createRobotSubsystems() {
        this.m_kitbotIntake = KitbotIntake.getInstance();
        this.m_kitbotLauncher = KitbotLauncher.getInstance();
    }

    /**
    * This method is used to set the default bindings for the swerve drive.  This should
    * only be overridden in rare cases.  This is only called once per robot type.
    */
    protected void configureSwerveBindings() {
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
                m_driveBase,
                m_constants
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
                    m_driveBase,
                    m_constants
                )
            );
        
        // Make gyroscope think current position is zero -- create button of driver controller
        m_driverController
            .create()
            .onTrue(new InstantCommand(m_driveBase::zeroGyro, m_driveBase));
    }

    @Override
    protected void configureBindings() {
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
    }

    @Override
    protected void createNamedCommands() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {}

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

    public class KitbotDrivebaseConstants {

    }
}
