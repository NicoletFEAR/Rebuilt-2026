// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.controllers.UniversalController;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.Utils;

public class LockRotationTowardsHub extends Command {
    /** Creates a new TeleopSwerve. */
    private SwerveDrive m_drivebase;

    private UniversalController m_driverController;

    private int m_throttleAxis;
    private int m_strafeAxis;

    private double m_throttle;
    private double m_strafe;
    private double m_steer;

    private boolean m_isOpenLoop;
    private boolean m_isFieldRelative;

    private double m_percentModifier;

    public LockRotationTowardsHub(
        UniversalController driverController,
        int throttleAxis,
        int strafeAxis,
        double percentModifier,
        boolean isOpenLoop,
        boolean isFieldRelative) {
        m_drivebase = SwerveDrive.getInstance();

        m_driverController = driverController;

        m_throttleAxis = throttleAxis;
        m_strafeAxis = strafeAxis;
        m_percentModifier = percentModifier;
        m_isOpenLoop = isOpenLoop;

        m_isFieldRelative = isFieldRelative;

        addRequirements(m_drivebase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    /**
     * This method is called repeatedly when this Command is scheduled to run.  It is used to 
     * apply changes to the throttle, strafe, and steer values based on the controller input.
     */
    @Override
    public void execute() {
        //isBatteryInBack gives value to change controls based on location of battery
        m_throttle = MathUtil.applyDeadband(
            DriveConstants.isBatteryInBack() * m_driverController.getRawAxis(m_throttleAxis),
            DriveConstants.getSwerveDeadband()
        );

        m_strafe = MathUtil.applyDeadband(
            DriveConstants.isBatteryInBack() * m_driverController.getRawAxis(m_strafeAxis),
            DriveConstants.getSwerveDeadband()
        );
        
        double desiredAngle = Math.atan2(
            DriveConstants.kHubPosition.getY() - m_drivebase.getPose().getTranslation().getY(),
            DriveConstants.kHubPosition.getX() - m_drivebase.getPose().getTranslation().getX()
        );

        if (Math.abs(desiredAngle - m_drivebase.getYaw().getRadians()) < DriveConstants.getRotationTolerance()) {
            m_steer = 0;    

        } else {
            double angleError = Utils.calculateShortestPath(m_drivebase.getYaw().getDegrees(), Math.toDegrees(desiredAngle));

            if (angleError > 0) {
                m_steer = 1;
            }
            else m_steer = -1;
        }

        m_throttle *= m_percentModifier;
        m_strafe *= m_percentModifier;
        m_steer *= m_percentModifier;

        m_drivebase.drive(m_throttle, m_strafe, m_steer, m_isOpenLoop, m_isFieldRelative);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
