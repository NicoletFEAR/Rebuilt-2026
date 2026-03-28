// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

public class LockRotationTowardsHub extends Command {
    /** Creates a new TeleopSwerve. */
    private SwerveDrive m_driveBase;

    private UniversalController m_driverController;

    private int m_throttleAxis;
    private int m_strafeAxis;

    private double m_throttle;
    private double m_strafe;
    private double m_steer;
    private PIDController m_steerController = new PIDController(DriveConstants.getAutoTargetKP(), DriveConstants.getAutoTargetKI(), DriveConstants.getAutoTargetKD());

    private Supplier<ChassisSpeeds> m_driveBaseSpeeds;
    private DoubleSupplier m_distanceToHub;
    private Hood m_hood;
    private Launcher m_launcher;

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
        Supplier<ChassisSpeeds> driveBaseSpeeds,
        DoubleSupplier distanceToHub,
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

        m_driveBaseSpeeds = driveBaseSpeeds;
        m_distanceToHub = distanceToHub;
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

        // Translation2d target;

        // if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
        //     if (m_driveBase.)
        //     target = DriveConstants.kBlueHubPosition;
        // } else {
        //     target = DriveConstants.kRedHubPosition;
        // }

        Translation2d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? DriveConstants.kBlueHubPosition
            : DriveConstants.kRedHubPosition;
        
        double estimatedFuelVelocity = Meters.convertFrom(3.0, Inches) * LauncherConstants.getLaunchVelocity() * Math.PI;
        double fuelVerticalVelocity = estimatedFuelVelocity * Math.sin(5.0 * Math.PI / 24.0);
        double fuelHorizontalVelocity = estimatedFuelVelocity * Math.cos(5.0 * Math.PI / 24.0);
        double timeToHub = (fuelVerticalVelocity + Math.sqrt(Math.pow(fuelVerticalVelocity, 2) + 19.62 * Meters.convertFrom(5.0, Feet))) / 9.81;
        ChassisSpeeds driveBaseSpeeds = m_driveBaseSpeeds.get();
        Logger.recordOutput("Time to hub", timeToHub);
        Logger.recordOutput("Fuel vertical velocity", fuelVerticalVelocity);
        // target = target.minus(new Translation2d(driveBaseSpeeds.vxMetersPerSecond, driveBaseSpeeds.vyMetersPerSecond).times(timeToHub));
        Logger.recordOutput("Rotation target", new Pose2d(target, new Rotation2d()));
        Pose2d drivePose = m_driveBase.getPose();
        double distanceToHub = m_driveBase.distanceToHub();
        Logger.recordOutput("Ball landing", new Pose2d(drivePose.getTranslation().plus(new Translation2d(distanceToHub * drivePose.getRotation().getCos() + driveBaseSpeeds.vxMetersPerSecond * timeToHub, distanceToHub * drivePose.getRotation().getSin() + driveBaseSpeeds.vyMetersPerSecond * timeToHub)), new Rotation2d()));
        double desiredAngle = Math.atan2(
            target.getY() - m_driveBase.getPose().getTranslation().getY(),
            target.getX() - m_driveBase.getPose().getTranslation().getX()
        );
        Logger.recordOutput("Ideal robot", new Pose2d(drivePose.getTranslation(), new Rotation2d(desiredAngle)));

        // Always run the PID — no dead zone cutoff that causes oscillation
        m_steer = m_steerController.calculate(m_driveBase.getYaw().getDegrees(), Math.toDegrees(desiredAngle)) / 180;

        m_throttle *= m_percentModifier;
        m_strafe *= m_percentModifier;
        m_steer *= m_percentModifier;

        m_driveBase.drive(m_throttle, m_strafe, m_steer, m_isOpenLoop, m_isFieldRelative);

        // Adjust hood angle based on distance to hub
        m_hood.runToPosition(MathUtil.clamp(
            LauncherConstants.kAutoAimHoodPositions.get(m_distanceToHub.getAsDouble()),
            LauncherConstants.getHoodMinPosition(),
            LauncherConstants.getHoodMaxPosition()
        ));

        // Adjust launcher speed based on distance to hub
        double speedModifier = MathUtil.clamp(
            LauncherConstants.kAutoAimSpeeds.get(m_distanceToHub.getAsDouble()),
            m_launcher.getMinSpeedModifier(), 1.0);
        speedModifier *= DriverStation.isAutonomous() ? 1.20 : 1.1;
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
