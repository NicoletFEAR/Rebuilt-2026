package frc.robot.containers;

import frc.robot.commands.TeleopSwerve;
import frc.robot.constants.GeneralConstants.GeneralOperatorConstants;
import frc.robot.constants.KitbotConstants.KitbotOperatorConstants;
import frc.robot.controllers.UniversalController;
import frc.robot.subsystems.kitbot.KitbotIntake;
import frc.robot.subsystems.kitbot.KitbotLauncher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class KitbotRobotContainer extends AbstractRobotContainer {

    protected final UniversalController m_driverController = new UniversalController(
        GeneralOperatorConstants.kDriverControllerPort, KitbotOperatorConstants.driveControllerType);

    private KitbotIntake m_kitbotIntake;
    private KitbotLauncher m_kitbotLauncher;

    public KitbotRobotContainer(Alliance alliance) {
       super(BotEnum.KITBOT, alliance);

        configureSwerveBindings();
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
        // Intakes fuel -- left bumper of driver controller
        m_driverController
            .L2()
            .onTrue(m_kitbotIntake.intake())
            .onTrue(m_kitbotLauncher.intake())
            .onFalse(m_kitbotIntake.off())
            .onFalse(m_kitbotLauncher.off());
            
        // launches fuel -- right bumper of driver controller
        m_driverController
            .R2()
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
