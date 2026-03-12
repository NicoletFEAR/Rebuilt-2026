package frc.robot.containers;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HadesRobotContainer extends AbstractRobotContainer {

    public HadesRobotContainer(Alliance alliance) {
        super(BotEnum.HADES, alliance);
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
}
