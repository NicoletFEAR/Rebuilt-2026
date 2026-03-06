package frc.robot.containers;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants;
import frc.robot.Constants.DeviceIds;
import frc.robot.commands.LockRotationTowardsHub;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.intake.IntakeDriver;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.launcher.Hood;
import frc.robot.subsystems.launcher.Indexer;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.led.Led;

/**
 * This class is used to encapsulate the robot code, including all hardware subsystems.  
 * In the case that new functionality should be added, it will likely need to added
 * to this class as it is the central point of all subsystems.
 */
public class TuskRobotContainer extends AbstractRobotContainer {
    
    private Launcher m_launcher;
    private Indexer m_indexer;
    private Hood m_hood;
    private IntakeDriver m_intakeDriver;
    private IntakePivot m_intakePivot;
    private Led m_led;
    private boolean m_limitOverrideMode = false;

    public TuskRobotContainer(Alliance alliance) {
        super(BotEnum.TUSK, alliance);
    }

    @Override
    protected void createRobotSubsystems() {
        this.m_launcher = new Launcher();
        this.m_indexer = new Indexer();
        this.m_hood = new Hood();
        this.m_intakeDriver = new IntakeDriver();
        this.m_intakePivot = new IntakePivot();
        this.m_led = new Led(DeviceIds.getLedID());
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
        
        // Enables Palantir-Class Target Lock on the hub
        m_driverController
            .R2()
            .whileTrue(
                new LockRotationTowardsHub(
                    m_driverController,
                    OperatorConstants.kThrottleAxis,
                    OperatorConstants.kStrafeAxis,
                    OperatorConstants.getDefaultSpeed(),
                    true,
                    true)
                .alongWith(m_launcher.adjustSpeedToHubDistance(m_driveBase::distanceToHub)
                    .alongWith(m_hood.adjustToHubDistance(m_driveBase::distanceToHub))))
            .onFalse(m_hood.endAutoTarget());
        
        new Trigger(this::aboutToSwitch)
            .onTrue(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 1.0))
                .alongWith(m_led.startScoringSwitchAnimation()))
            .onFalse(new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 0.0))
                .alongWith(m_led.startSwerveAnimation()));
        
        // Launches fuel by spinning up the launcher and then indexing the fuel -- right trigger of operator controller
        m_operatorController
            .R2()
            .onTrue(m_led
                .startLaunchAnimation()
                .alongWith(m_launcher.launch())
                .andThen(new WaitCommand(1))
                .andThen(m_indexer.index())
                .alongWith(m_intakePivot
                    .jostleOut()
                    .andThen(m_intakePivot
                        .in()
                        .until(m_intakePivot::isStuckOnBall)
                        .andThen(new InstantCommand(m_intakePivot::resetDesiredPosition))))
            ).onFalse(m_led.startSwerveAnimation()
                .alongWith(m_indexer.off())
                .alongWith(m_launcher.off()));
            
        // Increases launcher speed by 10% unless it's already at 100% -- b button on operator controller
        m_operatorController
            .circle()
            .onTrue(m_launcher.raiseSpeed());
        
        // Decreases launcher speed by 10% unless it's at 0% -- x button of operator controller
        m_operatorController
            .square()
            .onTrue(m_launcher.lowerSpeed());

        // Control the intake pivot manually -- left and right buttons on d-pad of operator controller
        m_intakePivot.setDefaultCommand(new RunCommand(() -> m_intakePivot.manualControl(() -> {
            if (m_operatorController.povRight().getAsBoolean() == m_operatorController.povLeft().getAsBoolean()) {
                return 0.0;
            } else if (m_operatorController.povRight().getAsBoolean()) {
                return 1.0;
            } else {
                return -1.0;
            }
        }, m_limitOverrideMode), m_intakePivot));

        // Control the hood manually -- up and down arrows of operator controller
        m_hood.setDefaultCommand(new RunCommand(() -> m_hood.manualControl(() -> {
            if (m_operatorController.povUp().getAsBoolean() == m_operatorController.povDown().getAsBoolean()) {
                return 0.0;
            } else if (m_operatorController.povUp().getAsBoolean()) {
                return 1.0;
            } else {
                return -1.0;
            }
        }, m_limitOverrideMode), m_hood));
        
        // Intakes fuel -- left trigger button on operator controller
        m_operatorController
            .L2()
            .onTrue(m_led.startIntakeAnimation()
                .alongWith(m_intakePivot.out())
                .alongWith(m_intakeDriver.intake()))
            .onFalse(m_led.startSwerveAnimation()
                .alongWith(m_intakePivot
                    .in()
                    .until(m_intakePivot::isStuckOnBall)
                    .andThen(new InstantCommand(m_intakePivot::resetDesiredPosition)))
                    .alongWith(m_intakeDriver.off()));

        // Disables limits for manual mechanism control -- create button on operator control
        m_operatorController
            .create()
            .onTrue(new InstantCommand(() -> m_limitOverrideMode = true))
            .onFalse(new InstantCommand(() -> m_limitOverrideMode = false));
    }

    @Override
    protected void createNamedCommands() {
        if (Constants.kRobotName.equals("tusk")) {
            // TODO: Replace these old commands with their newer versions in the autos
            NamedCommands.registerCommand("StartIntake", m_intakePivot.out().alongWith(m_intakeDriver.intake()));
            
            NamedCommands.registerCommand(
                "EndIntake",
                m_intakePivot.in().alongWith(m_intakeDriver.off())
            );

            NamedCommands.registerCommand("StartLaunch", m_indexer.index().alongWith(m_launcher.launch()));
            NamedCommands.registerCommand("EndLaunch", m_indexer.off().alongWith(m_launcher.off()));

            NamedCommands.registerCommand(
                "Launch",
                m_launcher
                    .launch()
                    .andThen(new WaitCommand(1))
                    .andThen(m_indexer.index())
                    .andThen(m_intakePivot.jostleOut()
                        .andThen(m_intakePivot
                            .in()
                            .until(m_intakePivot::isStuckOnBall)
                            .andThen(new InstantCommand(m_intakePivot::resetDesiredPosition))
                            .repeatedly())
                        .raceWith(new WaitCommand(1.25)))
                    .andThen(m_indexer.off().alongWith(m_launcher.off()))
            );
        }
    }

    public Command runAutoLedAnimation() {
        return m_led.startAutoAnimation();
    }

    public Command stopLedAnimation() {
        return m_led.stopAnimation();
    }

    public Alliance getAutoWinner() {
        String data = DriverStation.getGameSpecificMessage();

        if (data.length() > 0){
            switch (data.charAt(0)) {
                case 'R':
                    return Alliance.Red;
                case 'B':
                    return Alliance.Blue;
                default:
                    return null;
            }
        } else {
            return null;
        }
    }

    // TODO: Replace by the GameTimer class
    public boolean aboutToSwitch() {
        double matchTime = DriverStation.getMatchTime();

        return DriverStation.isTeleop()
            && DriverStation.isFMSAttached()
            && (
                (matchTime <= 131.0d && matchTime > 130.0d)
                || (matchTime <= 81.0d && matchTime > 80.0d)
                || (matchTime <= 56.0d && matchTime > 55.0d)
                || (matchTime <= 31.0d && matchTime > 30.0d)
                || (matchTime <= 1.0d && matchTime > 0.0d)
            );
    }

    /**
     * Ths mehtod should be called at the start of teleop and performs 
     * any necessary setup and processing for the period.  This includes determining 
     * who won autos, starting times, and sending alerts based on game shifts.
     */
    @Override
    public void teleopInit() {
        super.teleopInit();
        CommandScheduler.getInstance().schedule(m_led.startSwerveAnimation());
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
        Command autonomousCommand = getAutonomousCommand();
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(runAutoLedAnimation().alongWith(autonomousCommand));
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        CommandScheduler.getInstance().schedule(stopLedAnimation());
    }
}
