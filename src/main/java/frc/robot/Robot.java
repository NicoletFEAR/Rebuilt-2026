// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.Constants;
import frc.robot.robots.RobotIdentity;
import frc.robot.robots.base.RobotContainer;

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
    private final RobotIdentity m_identity = RobotIdentity.getIdentity();
    private final Alert m_unrecognizedRobot = new Alert("", AlertType.kError);
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        if (m_identity == RobotIdentity.UNRECOGNIZED) {
            m_unrecognizedRobot.setText(String.format(
                "Unrecognized robot (team number %d)",
                RobotController.getTeamNumber()
            ));
            m_unrecognizedRobot.set(true);
        }

        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("RobotName", m_identity.toString());

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
            // Log to a USB stick
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
            // When simulation starts, always be on Blue 1
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
            DriverStationSim.notifyNewData();
        }

        // Disables Hoot logging
        SignalLogger.enableAutoLogging(false);

        // Start logging! No more data receivers, replay sources, or metadata values may be added
        Logger.start();

        // WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        PortForwarder.add(5800, "10.47.86.11", 5800);

        RoboRioSim.setTeamNumber(4786);
        m_robotContainer = m_identity.getRobot();
        
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.periodic();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

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
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
