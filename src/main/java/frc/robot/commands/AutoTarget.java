// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.controllers.UniversalController;
import frc.robot.subsystems.launcher.Hood;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoTarget extends Command {
    /** Creates a new TeleopSwerve. */
    private SwerveDrive m_driveBase;

    private UniversalController m_driverController;

    private int m_throttleAxis;
    private int m_strafeAxis;

    private double m_throttle;
    private double m_strafe;
    private double m_steer;
    private PIDController m_steerController = new PIDController(DriveConstants.getAutoTargetKP(), DriveConstants.getAutoTargetKI(), DriveConstants.getAutoTargetKD());

    private Hood m_hood;
    private Launcher m_launcher;

    private boolean m_isOpenLoop;
    private boolean m_isFieldRelative;

    private double m_percentModifier;

    public AutoTarget (
        UniversalController driverController,
        int throttleAxis,
        int strafeAxis,
        double percentModifier,
        boolean isOpenLoop,
        boolean isFieldRelative,
        SwerveDrive driveBase,
        Hood hood,
        Launcher launcher) {
        m_driverController = driverController;

        m_throttleAxis = throttleAxis;
        m_strafeAxis = strafeAxis;
        m_percentModifier = percentModifier;
        m_isOpenLoop = isOpenLoop;

        m_steerController.enableContinuousInput(-180, 180);

        m_isFieldRelative = isFieldRelative;

        m_driveBase = driveBase;

        m_hood = hood;
        m_launcher = launcher;

        addRequirements(m_driveBase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_steerController.reset();
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

        Translation2d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? DriveConstants.kBlueHubPosition
            : DriveConstants.kRedHubPosition;

        ChassisSpeeds driveBaseSpeeds = m_driveBase.getFieldRelativeSpeeds();
        Pose2d drivePose = m_driveBase.getPose();

        double distance = Math.hypot(
            target.getY() - drivePose.getY(),
            target.getX() - drivePose.getX()
        );

        // Only compensate for the lateral (perpendicular) component of robot velocity
        // The radial component (toward/away from hub) is handled by launcher speed/hood
        Translation2d robotToHub = target.minus(drivePose.getTranslation());
        Translation2d robotToHubUnit = robotToHub.div(distance);
        Translation2d velocity = new Translation2d(driveBaseSpeeds.vxMetersPerSecond, driveBaseSpeeds.vyMetersPerSecond);

        // Project velocity onto the perpendicular of the robot-to-hub direction
        // perpendicular = (-hubUnit.y, hubUnit.x)
        double lateralSpeed = velocity.getX() * (-robotToHubUnit.getY()) + velocity.getY() * robotToHubUnit.getX();
        Translation2d lateralVelocity = new Translation2d(-robotToHubUnit.getY(), robotToHubUnit.getX()).times(lateralSpeed);

        Translation2d velocityCompensation = lateralVelocity.times(LauncherConstants.kAutoAimTof.get(distance));
        target = target.minus(velocityCompensation);
        Logger.recordOutput("Rotation target", new Pose2d(target, new Rotation2d()));

        distance = Math.hypot(
            target.getY() - drivePose.getY(),
            target.getX() - drivePose.getX()
        );

        double desiredAngle = Math.atan2(
            target.getY() - drivePose.getY(),
            target.getX() - drivePose.getX()
        );

        // Always run the PID — no dead zone cutoff that causes oscillation
        m_steer = m_steerController.calculate(m_driveBase.getYaw().getDegrees(), Math.toDegrees(desiredAngle)) / 180;

        m_throttle *= m_percentModifier;
        m_strafe *= m_percentModifier;
        m_steer *= m_percentModifier;

        m_driveBase.drive(m_throttle, m_strafe, m_steer, m_isOpenLoop, m_isFieldRelative);

        // Adjust hood angle based on distance to hub
        m_hood.runToPosition(MathUtil.clamp(
            LauncherConstants.kAutoAimHoodPositions.get(distance),
            LauncherConstants.getHoodMinPosition(),
            LauncherConstants.getHoodMaxPosition()
        ));

        // Adjust launcher speed based on distance to hub
        double speedModifier = MathUtil.clamp(
            LauncherConstants.kAutoAimSpeeds.get(distance),
            m_launcher.getMinSpeedModifier(), 1.0);
        speedModifier *= 1.25;
        m_launcher.setSpeedModifierDirect(speedModifier);
        m_launcher.setVelocity(LauncherConstants.getLaunchVelocity() * speedModifier);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(m_hood.endAutoTarget());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
