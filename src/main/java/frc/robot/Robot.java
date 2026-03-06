// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.containers.AbstractRobotContainer;
import frc.robot.containers.KitbotRobotContainer;
import frc.robot.containers.TuskRobotContainer;
import frc.robot.containers.HadesRobotContainer;
import frc.robot.containers.BotEnum;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;

/**
 * The system is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    
    private AbstractRobotContainer m_robotContainer;
    private double m_autoStart;
    private boolean m_printedAutoTiming = false;

    @Override
    public void robotInit() {
        Constants.instantiateProperties();

        // Various values that make managing the code version easier
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("RobotName", Constants.kRobotName);

        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            case -1:
                Logger.recordMetadata("GitDirty", "Error");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        if (isReal()) {
            // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new WPILOGWriter());
            // Publish data to NetworkTables
            Logger.addDataReceiver(new NT4Publisher());

        } 
        else if (Constants.kIsReplay) {
            // When replaying on a laptop, run unconstrained by RoboRIO hardware limitations
            setUseTiming(false);
            // Gets the path to the log file open in AdvantageScope
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        } 
        else {
            // We don't need to keep log files during simulation
            Logger.addDataReceiver(new NT4Publisher());
        }

        // Disables Hoot logging
        SignalLogger.enableAutoLogging(false);

        // Start logging! No more data receivers, replay sources, or metadata values may be added.
        Logger.start();

        // TODO Verify if the web server is actually required
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        createRobotContainer();

        RoboRioSim.setTeamNumber(BotEnum.TUSK.getTeamNumber());
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    public static int getTeamNumber() {
        if (Robot.isReal())
            return RobotController.getTeamNumber();
        else
            return BotEnum.TUSK.getTeamNumber();
    }

    public static Alliance getAlliance() {
        Alliance alliance = null;

        try {
            alliance = DriverStation.getAlliance().get();
            if (alliance == null)
                alliance = Alliance.Blue;
        }
        catch (Exception e) {
            alliance = Alliance.Blue;
        }

        return alliance;
    }

    private BotEnum getBotEnum() {
        return BotEnum.fromTeamNumber(getTeamNumber());
    }

    private void createRobotContainer() {
        switch (getBotEnum()) {
            case KITBOT:
                this.m_robotContainer = new KitbotRobotContainer(getAlliance());
                break;
            case HADES:
                this.m_robotContainer = new HadesRobotContainer(getAlliance());
                break;
            case TUSK:
            default:
                this.m_robotContainer = new TuskRobotContainer(getAlliance());
                break;
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Command command = null;
        
        if (m_robotContainer != null)
            command = m_robotContainer.getAutonomousCommand();

        if (command != null) {
            if (!command.isScheduled() && !m_printedAutoTiming) {
                if (DriverStation.isAutonomousEnabled()) {
                    System.out.println("Auto finished in " + (Timer.getTimestamp() - m_autoStart) + " seconds");
                } else {
                    System.out.println("Auto cancelled in " + (Timer.getTimestamp() - m_autoStart) + " seconds");
                }
                m_printedAutoTiming = true;
            }
        }

        if (m_robotContainer != null)
            m_robotContainer.periodic();
    }

    @Override
    public void disabledInit() {
        if (m_robotContainer != null)
            m_robotContainer.disabledInit();
    }

    @Override
    public void disabledPeriodic() {
        if (m_robotContainer != null)
            m_robotContainer.disabledPeriodic();
    }

    @Override
    public void disabledExit() {
        if (m_robotContainer != null)
            m_robotContainer.disabledExit();
    }

    @Override
    public void autonomousInit() {
        m_autoStart = Timer.getTimestamp();
        if (m_robotContainer != null)
            m_robotContainer.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {
        if (m_robotContainer != null)
            m_robotContainer.autonomousPeriodic();
    }

    @Override
    public void autonomousExit() {
        if (m_robotContainer != null)
            m_robotContainer.autonomousExit();
    }

    @Override
    public void teleopInit() {
        if (m_robotContainer != null)
            m_robotContainer.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        if (m_robotContainer != null)
            m_robotContainer.teleopPeriodic();
    }

    @Override
    public void teleopExit() {
        if (m_robotContainer != null)
            m_robotContainer.teleopExit();
    }

    @Override
    public void testInit() {
        if (m_robotContainer != null)
            m_robotContainer.testInit();
    }

    @Override
    public void testPeriodic() {
        if (m_robotContainer != null)
            m_robotContainer.testPeriodic();
    }

    @Override
    public void testExit() {
        if (m_robotContainer != null)
            m_robotContainer.testExit();
    }
}
