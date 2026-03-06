package frc.robot.containers;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.kitbot.KitbotIntake;
import frc.robot.subsystems.kitbot.KitbotLauncher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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

        // Make gyroscope think current position is zero -- create button of driver controller
        m_driverController
            .create()
            .onTrue(new InstantCommand(m_driveBase::zeroGyro, m_driveBase));
        
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
    protected void createNamedCommands() {
        // TODO Auto-generated method stub
        
    }
    
}
