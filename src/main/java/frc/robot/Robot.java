// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
    private RobotContainer m_robotContainer;
    private double m_autoStart;
    private boolean m_printedAutoTiming = false;
    private Command m_autonomousCommand;

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
        } else if (Constants.kIsReplay) {
            // When replaying on a laptop, run unconstrained by RoboRIO hardware limitations
            setUseTiming(false);
            // Gets the path to the log file open in AdvantageScope
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        } else {
            // We don't need to keep log files during simulation
            Logger.addDataReceiver(new NT4Publisher());
        }

        // Disables Hoot logging
        SignalLogger.enableAutoLogging(false);

        // Start logging! No more data receivers, replay sources, or metadata values may be added.
        Logger.start();

        // WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        PortForwarder.add(5800, "10.47.86.11", 5800);

        // It's obvious if a joystick is disconnected and I'm tired of these warnings
        // clogging up the driver station
        DriverStation.silenceJoystickConnectionWarning(true);

        RoboRioSim.setTeamNumber(4786);
        m_robotContainer = new RobotContainer();

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.periodic();

        if (m_autonomousCommand != null) {
            if (!m_autonomousCommand.isScheduled() && !m_printedAutoTiming) {
                if (DriverStation.isAutonomousEnabled()) {
                    System.out.println("Auto finished in " + (Timer.getTimestamp() - m_autoStart) + " seconds");
                } else {
                    System.out.println("Auto cancelled in " + (Timer.getTimestamp() - m_autoStart) + " seconds");
                }

                CommandScheduler.getInstance().schedule(m_robotContainer.stopLedAnimation());
                m_printedAutoTiming = true;
            }
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autoStart = Timer.getTimestamp();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_robotContainer.autonomousInitCommand().andThen(m_robotContainer.adjustLauncherSpeedToHub().alongWith(m_autonomousCommand)));
        }

        m_robotContainer.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        m_robotContainer.autonomousExit();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_robotContainer.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        m_robotContainer.teleopPeriodic();
    }

    @Override
    public void teleopExit() {
        m_robotContainer.teleopExit();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
