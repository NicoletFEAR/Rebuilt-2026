package frc.robot.containers;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * This class is used to encapsulate the robot code, including all hardware subsystems.  
 * In the case that new functionality should be added, it will likely need to added
 * to this class as it is the central point of all subsystems.
 */
public abstract class AbstractRobotContainer {

    // Common items for a robot
    private static BotEnum m_botEnum;
    protected final Alliance m_alliance;
    protected SendableChooser<Command> m_autoChooser;
    protected SwerveDrive m_driveBase;


    /**
     * This constructor will create the robot container and the common setup needed for a robot
     * @param botEnum The Type of robot to create
     * @param alliance The alliance that the robot is on
     */
    public AbstractRobotContainer(BotEnum botEnum, Alliance alliance) {
        // Call the super's constructor first
        super();

        // Set the bot type
        this.m_botEnum = botEnum;

        // Set the alliance
        if (alliance == null)
            this.m_alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        else
            this.m_alliance = alliance;

        // Perform the setup of the bot
        createDriveBase();
        createRobotSubsystems();
        createNamedCommands();
        configureBindings();
        this.m_autoChooser = AutoBuilder.buildAutoChooser();

        Shuffleboard.getTab("Main").add("Auto Chooser", m_autoChooser).withPosition(5, 0).withSize(5, 2);
    }

    /**P
     * This method is called when the robot container is created.  This is used to
     * create the drive base for the robot.  This is only called once
     * per robot type.
     */
    protected abstract void createDriveBase();
    

    /**
     * This method is called when the robot container is created.  This is used to
     * create all of the subsystems for the robot.  This is only called once
     * per robot type.
     */
    protected abstract void createRobotSubsystems();

    /**
     * This method is called when the robot container is created.  This is used to 
     * create all of the named commands for the robot.  This is only called once
     * per robot type.
     */
    protected abstract void createNamedCommands();
    
    /**
     * This method is called when the robot container is created.  This is used to
     * configure the button bindings for the robot.  This is called every time
     * the robot is enabled.
     */
    protected abstract void configureBindings();

    /**
     * This method will return the command to run in Autonomous
     * @return The Command to execute
     */
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    /**
     * This method will return the alliance that the robot is on
     * @return Alliance
     */
    public Alliance getAlliance() {
        return m_alliance;
    }

    /**
     * This method will be called on a periodic basis and used to 
     * update dashboards and other periodic tasks.  This implementation includes
     * setting the alliance and displaying the Match Time on the dashboard.
     */
    public void periodic() {
        SmartDashboard.putString("Alliance", m_alliance.toString());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    /**
     * This method should be called at the start of autonomous and perform 
     * any necessary setup and processing for the autonomous period
     */
    public abstract void autonomousInit();

    /**
     * This method is called periodically durring autonomous
     */
    public abstract void autonomousPeriodic();

    /**
     * This method should be called at the end of autonomous and perform
     * any necessary cleanup for the autonomous period
     */
    public abstract void autonomousExit();

    /**
     * Ths mehtod should be called at the start of teleop and performs 
     * any necessary setup and processing for the period.  This includes determining 
     * who won autos, starting times, and sending alerts based on game shifts.
     */
    public void teleopInit() {
        Command command = getAutonomousCommand();
        if (command != null)
            command.cancel();
    }

    /**
     * This methid should be called at internvals in the teleop period.  This 
     * should not be used for robot functionality, but instead used to update
     * dashboards
     */
    public abstract void teleopPeriodic();

    /**
     * This method should be called at the end of teleop and perform
     * any necessary cleanup for the teleop period
     */
    public abstract void teleopExit();

    /**
     * This method should be called when the robot is disabled and perform any necessary
     * cleanup for the disabled period
     */
    public abstract void disabledInit();

    /**
     * This method should be called periodically while the robot is disabled
     */
    public abstract void disabledPeriodic();

    /**
     * This method should be called at the end of disabled and perform
     * any necessary cleanup for the disabled period
     */
    public abstract void disabledExit();

    /**
     * This method should be called at the start of test mode and perform any necessary
     * setup and processing for the test period
     */
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This method should be called periodically while the robot is in test mode
     */
    public abstract void testPeriodic();

    /**
     * This method should be called at the end of test mode and perform
     * any necessary cleanup for the test period
     */
    public abstract void testExit();

    public BotEnum getBot() {
        return m_botEnum;
    }
}
