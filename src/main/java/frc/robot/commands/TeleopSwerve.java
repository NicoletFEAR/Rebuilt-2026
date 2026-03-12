// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.data.controller.Controller;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TeleopSwerve extends Command {
    /** Creates a new TeleopSwerve. */
    private SwerveDrive m_driveBase;

    private Controller m_driverController;

    private double m_throttle;
    private double m_strafe;
    private double m_steer;

    private boolean m_isOpenLoop;
    private boolean m_isFieldRelative;

    private double m_percentModifier;

    public TeleopSwerve(
        Controller driverController,
        double percentModifier,
        boolean isOpenLoop,
        boolean isFieldRelative,
        SwerveDrive driveBase) {
        m_driverController = driverController;

        m_percentModifier = percentModifier;
        m_isOpenLoop = isOpenLoop;

        m_isFieldRelative = isFieldRelative;

        m_driveBase = driveBase;

        addRequirements(m_driveBase);
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
            DriveConstants.isBatteryInBack() * m_driverController.leftY(),
            DriveConstants.getSwerveDeadband()
        );

        m_strafe = MathUtil.applyDeadband(
            DriveConstants.isBatteryInBack() * m_driverController.leftX(),
            DriveConstants.getSwerveDeadband()
        );

        m_steer = MathUtil.applyDeadband(
            DriveConstants.isBatteryInBack() * m_driverController.rightX(),
            DriveConstants.getSwerveDeadband()
        );

        m_throttle *= m_percentModifier;
        m_strafe *= m_percentModifier;
        m_steer *= m_percentModifier;

        m_driveBase.drive(m_throttle, m_strafe, m_steer, m_isOpenLoop, m_isFieldRelative);
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
