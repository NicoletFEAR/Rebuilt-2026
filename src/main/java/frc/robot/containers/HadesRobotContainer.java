package frc.robot.containers;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.constants.GeneralConstants.GeneralOperatorConstants;
import frc.robot.constants.HadesConstants.HadesOperatorConstants;
import frc.robot.controllers.UniversalController;

public class HadesRobotContainer extends AbstractRobotContainer {

    protected final UniversalController m_driverController = new UniversalController(
        GeneralOperatorConstants.kDriverControllerPort, HadesOperatorConstants.driveControllerType);

    public HadesRobotContainer(Alliance alliance) {
        super(BotEnum.HADES, alliance);

        configureSwerveBindings();
    }

    @Override
    protected void configureBindings() {}

    @Override
    protected void createNamedCommands() {}

    @Override
    protected void createRobotSubsystems() {}

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

    /**
    * This method is used to set the default bindings for the swerve drive.  This should
    * only be overridden in rare cases.  This is only called once per robot type.
    */
    protected void configureSwerveBindings() {
        // Driving -- joysticks of driver controller
        m_driveBase.setDefaultCommand(
            new TeleopSwerve(
                m_driverController,
                HadesOperatorConstants.kThrottleAxis,
                HadesOperatorConstants.kStrafeAxis,
                HadesOperatorConstants.kSteerAxis,
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
                    HadesOperatorConstants.kThrottleAxis,
                    HadesOperatorConstants.kStrafeAxis,
                    HadesOperatorConstants.kSteerAxis,
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
}
