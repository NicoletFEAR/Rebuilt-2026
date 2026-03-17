// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.HadesConstants.HadesDriveConstants;
import frc.robot.constants.HadesConstants.HadesGeneralConstants;
import frc.robot.constants.KitbotConstants.KitbotDriveConstants;
import frc.robot.constants.KitbotConstants.KitbotGeneralConstants;
import frc.robot.constants.TuskConstants.TuskDriveConstants;
import frc.robot.constants.TuskConstants.TuskGeneralConstants;
import frc.robot.containers.AbstractRobotContainer;
import frc.robot.containers.BotEnum;
import frc.robot.controllers.UniversalController;
import frc.robot.subsystems.swerve.SwerveDrive;

public class LockRotationTowardsHub extends Command {
    /** Creates a new TeleopSwerve. */
    private SwerveDrive m_driveBase;

    private UniversalController m_driverController;

    private int m_throttleAxis;
    private int m_strafeAxis;

    private double m_throttle;
    private double m_strafe;
    private double m_steer;
    private PIDController m_steerController = new PIDController(TuskDriveConstants.getAutoTargetKP, TuskDriveConstants.getAutoTargetKI, TuskDriveConstants.getAutoTargetKD);


    private boolean m_isOpenLoop;
    private boolean m_isFieldRelative;

    private double m_percentModifier;

    public LockRotationTowardsHub(
        UniversalController driverController,
        int throttleAxis,
        int strafeAxis,
        double percentModifier,
        boolean isOpenLoop,
        boolean isFieldRelative,
        SwerveDrive driveBase) {
        m_driverController = driverController;

        m_throttleAxis = throttleAxis;
        m_strafeAxis = strafeAxis;
        m_percentModifier = percentModifier;
        m_isOpenLoop = isOpenLoop;

        m_steerController.enableContinuousInput(-180, 180);

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
        if (AbstractRobotContainer.getBot() == BotEnum.TUSK) {
            //isBatteryInBack gives value to change controls based on location of battery
            m_throttle = MathUtil.applyDeadband(
                TuskGeneralConstants.isBatteryInBack() * m_driverController.getRawAxis(m_throttleAxis),
                TuskDriveConstants.kSwerveDeadband
            );

            m_strafe = MathUtil.applyDeadband(
                TuskGeneralConstants.isBatteryInBack() * m_driverController.getRawAxis(m_strafeAxis),
                TuskDriveConstants.kSwerveDeadband
            );
        } else if (AbstractRobotContainer.getBot() == BotEnum.HADES) {
            //isBatteryInBack gives value to change controls based on location of battery
            m_throttle = MathUtil.applyDeadband(
                HadesGeneralConstants.isBatteryInBack() * m_driverController.getRawAxis(m_throttleAxis),
                HadesDriveConstants.kSwerveDeadband
            );

            m_strafe = MathUtil.applyDeadband(
                HadesGeneralConstants.isBatteryInBack() * m_driverController.getRawAxis(m_strafeAxis),
                HadesDriveConstants.kSwerveDeadband
            );
        } else {
            //isBatteryInBack gives value to change controls based on location of battery
            m_throttle = MathUtil.applyDeadband(
                KitbotGeneralConstants.isBatteryInBack() * m_driverController.getRawAxis(m_throttleAxis),
                KitbotDriveConstants.kSwerveDeadband
            );

            m_strafe = MathUtil.applyDeadband(
                KitbotGeneralConstants.isBatteryInBack() * m_driverController.getRawAxis(m_strafeAxis),
                KitbotDriveConstants.kSwerveDeadband
            );
        }
        
        double desiredAngle = Math.atan2(
            GeneralConstants.kHubPosition.getY() - m_driveBase.getPose().getTranslation().getY(),
            GeneralConstants.kHubPosition.getX() - m_driveBase.getPose().getTranslation().getX()
        );

        if (AbstractRobotContainer.getBot() == BotEnum.TUSK) {
            if (Math.abs(desiredAngle - m_driveBase.getYaw().getRadians()) < TuskDriveConstants.rotationTolerance) {
                m_steer = 0;    

            } else {
                m_steer = m_steerController.calculate(m_driveBase.getYaw().getDegrees(), Math.toDegrees(desiredAngle)) / 180;
            }
        } else if (AbstractRobotContainer.getBot() == BotEnum.HADES) {
            if (Math.abs(desiredAngle - m_driveBase.getYaw().getRadians()) < HadesDriveConstants.rotationTolerance) {
                m_steer = 0;    

            } else {
                m_steer = m_steerController.calculate(m_driveBase.getYaw().getDegrees(), Math.toDegrees(desiredAngle)) / 180;
            }
        } else {
            if (Math.abs(desiredAngle - m_driveBase.getYaw().getRadians()) < KitbotDriveConstants.rotationTolerance) {
                m_steer = 0;    

            } else {
                m_steer = m_steerController.calculate(m_driveBase.getYaw().getDegrees(), Math.toDegrees(desiredAngle)) / 180;
            }
        }

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
