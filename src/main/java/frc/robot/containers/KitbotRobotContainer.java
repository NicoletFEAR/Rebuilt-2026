package frc.robot.containers;

import frc.robot.subsystems.kitbot.KitbotIntake;
import frc.robot.subsystems.kitbot.KitbotLauncher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class KitbotRobotContainer extends AbstractRobotContainer {

    private KitbotIntake m_kitbotIntake;
    private KitbotLauncher m_kitbotLauncher;

    public KitbotRobotContainer(Alliance alliance) {
        super(BotEnum.KITBOT, alliance);
    }

    @Override
    protected void createRobotSubsystems() {
        this.m_kitbotIntake = KitbotIntake.getInstance();
        this.m_kitbotLauncher = KitbotLauncher.getInstance();
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
    
}
