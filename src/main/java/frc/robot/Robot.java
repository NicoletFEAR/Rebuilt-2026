// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
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

/**
 * The system is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the {@link edu.wpi.first.wpilibj.TimedRobot} documentation. If you change
 * the name of this class or the package after creating this project, you must also update the
 * build.gradle file in the project.
 */
public class Robot extends LoggedRobot {
    private RobotIdentity m_identity;
    private final Alert m_unrecognizedRobot = new Alert("", AlertType.kError);
    private RobotContainer m_robotContainer;

    @Override
    public void disabledPeriodic() {}

    @Override
    public void robotInit() {
        RoboRioSim.setTeamNumber(4786);
        m_identity = RobotIdentity.getIdentity();

        if (m_identity == RobotIdentity.UNRECOGNIZED) {
            m_unrecognizedRobot.setText(String.format(
                "Unrecognized robot (team number %d)",
                RobotController.getTeamNumber()
            ));
            m_unrecognizedRobot.set(true);
        } else {
            m_unrecognizedRobot.set(false);
        }

        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("RobotName", m_identity.toString());

        Logger.recordMetadata("GitDirty", switch(BuildConstants.DIRTY) {
            case 0 -> "All changes committed";
            case 1 -> "Uncommitted changes";
            case -1 -> "Error";
            default -> "Unknown";
        });

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else if (Constants.kIsReplay) {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        } else {
            Logger.addDataReceiver(new NT4Publisher());
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
            DriverStationSim.notifyNewData();
        }

        SignalLogger.enableAutoLogging(false);
        Logger.start();

        DriverStation.silenceJoystickConnectionWarning(true);

        m_robotContainer = m_identity.getRobot();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.periodic();
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
