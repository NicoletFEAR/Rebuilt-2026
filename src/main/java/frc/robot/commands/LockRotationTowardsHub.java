// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

// import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SwerveConstantsInterface;
import frc.robot.controllers.UniversalController;
import frc.robot.subsystems.swerve.SwerveDrive;

public class LockRotationTowardsHub extends Command {
    
    private SwerveConstantsInterface m_constants;
    private SwerveDrive m_driveBase;
    private UniversalController m_driverController;
    private int m_throttleAxis;
    private int m_strafeAxis;
    private double m_throttle;
    private double m_strafe;
    private double m_steer;
    private PIDController m_steerController;
    private DoubleSupplier m_launcherVelocity;
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
        SwerveDrive driveBase,
        DoubleSupplier launcherVelocity,
        SwerveConstantsInterface constants) {
        
        this.m_driverController = driverController;
        this.m_throttleAxis = throttleAxis;
        this.m_strafeAxis = strafeAxis;
        this.m_percentModifier = percentModifier;
        this.m_isOpenLoop = isOpenLoop;
        this.m_constants = constants;
        this.m_isFieldRelative = isFieldRelative;
        this.m_driveBase = driveBase;
        this.m_launcherVelocity = launcherVelocity;
        this.m_steerController = new PIDController(m_constants.getAutoTargetKP(), m_constants.getAutoTargetKI(), m_constants.getAutoTargetKD());
        m_steerController.enableContinuousInput(-180, 180);

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
            m_constants.getIsBatteryInBack() * m_driverController.getRawAxis(m_throttleAxis),
            m_constants.getSwerveDeadband()
        );

        m_strafe = MathUtil.applyDeadband(
            m_constants.getIsBatteryInBack() * m_driverController.getRawAxis(m_strafeAxis),
            m_constants.getSwerveDeadband()
        );

        Translation2d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? GeneralConstants.kBlueHubPosition
            : GeneralConstants.kRedHubPosition;
        
        // double estimatedFuelVelocity = Meters.convertFrom(3.0, Inches) * m_launcherVelocity.getAsDouble() * Math.PI;
        // double timeToHub = 2.0 * estimatedFuelVelocity / 19.6;
        
        double desiredAngle = Math.atan2(
            target.getY() - m_driveBase.getPose().getTranslation().getY(),
            target.getX() - m_driveBase.getPose().getTranslation().getX()
        );

        if (Math.abs(desiredAngle - m_driveBase.getYaw().getRadians()) < GeneralConstants.rotationTolerance) {
            m_steer = 0;    

        } else {
            m_steer = m_steerController.calculate(m_driveBase.getYaw().getDegrees(), Math.toDegrees(desiredAngle)) / 180;
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
