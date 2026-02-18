// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;

import frc.robot.util.Utils;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {

    private static SwerveDrive m_instance = null;

    private SwerveModule[] m_modules;

    private Pigeon2 m_pigeon;

    private SwerveDriveKinematics m_kinematics;
    private SwerveDrivePoseEstimator m_poseEstimator;

    private SwerveDrivePoseEstimator m_megaTag1PoseEstimator;

    private SwerveModuleState[] m_desiredModuleStates;
    private SwerveModulePosition[] m_modulePositions;
    private ChassisSpeeds m_chassisSpeeds;

    private DriveMode m_driveMode = DriveMode.TELEOP;

    private Field2d m_field;

    private double m_simYaw;

    private double m_angleToSnap = Double.POSITIVE_INFINITY;

    private PIDController angleController = new PIDController(.016, 0.003, 0.0);

    public SwerveDrive() {
        if (DriveConstants.usesDriveKrakens()) {
            m_modules = new SwerveModule[] {
                new TalonSwerveModule(DriveConstants.kFrontLeft),
                new TalonSwerveModule(DriveConstants.kFrontRight),
                new TalonSwerveModule(DriveConstants.kBackLeft),
                new TalonSwerveModule(DriveConstants.kBackRight)
            };
        } else {
            m_modules = new SwerveModule[] {
                new SparkMaxSwerveModule(DriveConstants.kFrontLeft),
                new SparkMaxSwerveModule(DriveConstants.kFrontRight),
                new SparkMaxSwerveModule(DriveConstants.kBackLeft),
                new SparkMaxSwerveModule(DriveConstants.kBackRight)
            };
        }

        m_pigeon = new Pigeon2(DeviceIds.getPigeonId(), new CANBus(Constants.hasCANivore() ? "*" : "rio"));

        m_pigeon.getConfigurator().apply(new Pigeon2Configuration().GyroTrim.withGyroScalarZ(1));

        m_kinematics = new SwerveDriveKinematics(DriveConstants.kModuleTranslations);

        m_desiredModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };

        m_modulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };

        m_chassisSpeeds = new ChassisSpeeds();

        Timer.delay(1.0);
        resetModulesToAbsolute();

        m_poseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                RobotContainer.getAlliance() == Alliance.Blue ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.0),
                VecBuilder.fill(0.9, 0.9, 9999999));

        m_megaTag1PoseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                RobotContainer.getAlliance() == Alliance.Blue ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.0001),
                VecBuilder.fill(0.9, 0.9, .5));

        m_pigeon.setYaw(0);

        m_field = new Field2d();
        RobotContainer.m_mainTab.add(m_field).withPosition(6, 2).withSize(6, 3);

        angleController.setTolerance(1);

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> driveRobotRelative(speeds),

                new PPHolonomicDriveController(
                        new PIDConstants(DriveConstants.getDriveKP(), DriveConstants.getDriveKI(), DriveConstants.getDriveKD()),
                        new PIDConstants(DriveConstants.getTurnKP(), DriveConstants.getTurnKI(), DriveConstants.getTurnKD())),

                DriveConstants.kRobotConfig,

                () -> {
                    var alliancenew = DriverStation.getAlliance();
                    if (alliancenew.isPresent()) {
                        return alliancenew.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },

                this);
    }

    public static SwerveDrive getInstance() {
        if (m_instance == null) {
            m_instance = new SwerveDrive();
        }

        return m_instance;
    }

    public SwerveModulePosition[] getModulePositions() {
        for (int i = 0; i < m_modules.length; i++) {
            m_modulePositions[i] = m_modules[i].getModulePosition();
        }

        return m_modulePositions;
    }

    public SwerveModuleState[] getModuleStates() {
        for (int i = 0; i < m_modules.length; i++) {
            m_desiredModuleStates[i] = m_modules[i].getModuleState();
        }

        return m_desiredModuleStates;
    }

    public Pose2d getPose() {
        return new Pose2d(m_poseEstimator.getEstimatedPosition().getTranslation(), getYaw());
    }

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        // return m_chassisSpeeds;
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(m_chassisSpeeds, getYaw());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        setModuleStates(m_kinematics.toSwerveModuleStates(speeds), false);
        m_chassisSpeeds = speeds;

        m_simYaw += Units.radiansToDegrees(speeds.omegaRadiansPerSecond * Constants.kdt);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw());
        setModuleStates(m_kinematics.toSwerveModuleStates(m_chassisSpeeds), false);

        m_simYaw += Units.radiansToDegrees(speeds.omegaRadiansPerSecond * Constants.kdt);
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule module : m_modules) {
            module.resetAngleToAbsolute();
        }
    }

    public void setModuleStates(boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                m_desiredModuleStates, DriveConstants.getMaxModuleSpeed());

        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setSwerveModuleState(m_desiredModuleStates[i], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] moduleStates, boolean isOpenLoop) {
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setSwerveModuleState(moduleStates[i], isOpenLoop);
        }
    }

    public double getRotationsInDegrees() {
        if (getYaw().getDegrees() < 0) {
            return (getYaw().getDegrees() % -360) + 360;
        }
        return getYaw().getDegrees() % 360;
    }

    public Rotation2d getPigeonYaw() {
        if (RobotBase.isReal()) {
            Rotation2d rotation = RobotContainer.getAlliance() == Alliance.Blue ? m_pigeon.getRotation2d()
                    : m_pigeon.getRotation2d().plus(Rotation2d.fromDegrees(180));

            return rotation;
        } else {
            return Rotation2d.fromDegrees(m_simYaw);
        }
    }

    public Rotation2d getYaw() {
        if (DriverStation.isAutonomous()) {
            return getPigeonYaw();
        } else {
            return m_megaTag1PoseEstimator.getEstimatedPosition().getRotation();
        }
    }

    public void setDriveMode(DriveMode driveMode) {
        m_driveMode = driveMode;
    }

    public void setAngleToSnap(double angleToSnap) {
        m_angleToSnap = angleToSnap;
    }

    public void driveClosedLoop(double throttle, double strafe, double steer) {

        m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, steer, getYaw());

        m_chassisSpeeds = ChassisSpeeds.discretize(m_chassisSpeeds, Constants.kdt);

        m_desiredModuleStates = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

        setModuleStates(m_desiredModuleStates, false);

        if (RobotBase.isSimulation())
            m_simYaw += Units.radiansToDegrees(m_chassisSpeeds.omegaRadiansPerSecond * Constants.kdt);

    }

    public void drive(
            double throttle, double strafe, double steer, boolean isOpenLoop, boolean isFieldRelative) {

        if (throttle + strafe + steer != 0 && m_driveMode == DriveMode.XWHEELS) {
            m_driveMode = DriveMode.TELEOP;
        }

        switch (m_driveMode) {
            case TELEOP:
                throttle *= DriveConstants.getMaxModuleSpeed();
                strafe *= DriveConstants.getMaxModuleSpeed();

                if (m_angleToSnap != Double.POSITIVE_INFINITY) {
                    steer = angleController.calculate(
                            Utils.getAdjustedYawDegrees(getYaw().getDegrees(), m_angleToSnap), 180);
                    steer *= DriveConstants.getMaxModuleSpeed();

                    if (angleController.atSetpoint())
                        m_angleToSnap = Double.POSITIVE_INFINITY;
                }

                steer *= DriveConstants.getMaxModuleSpeed();

                m_chassisSpeeds = isFieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, steer, getYaw())
                        : new ChassisSpeeds(throttle, strafe, steer);

                m_chassisSpeeds = ChassisSpeeds.discretize(m_chassisSpeeds, Constants.kdt);

                m_desiredModuleStates = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

                if (isOpenLoop) {
                    setModuleStates(m_desiredModuleStates, isOpenLoop);
                } else {
                    setModuleStates(m_desiredModuleStates, isOpenLoop);
                }

                if (RobotBase.isSimulation())
                    m_simYaw += Units.radiansToDegrees(m_chassisSpeeds.omegaRadiansPerSecond * Constants.kdt);

                break;
            case XWHEELS:
                Utils.copyModuleStates(DriveConstants.kXWheels, m_desiredModuleStates);
                setModuleStates(isOpenLoop);
                break;
        }
    }

    public void runVolts(Voltage volts) {
        for (SwerveModule module : m_modules) {
            module.runVolts(volts, 0);
        }
    }

    public Command characterizeDrivebase(BooleanSupplier finishRoutine) {
        var sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage), null, this));

        return new SequentialCommandGroup(
                new PrintCommand("Starting"),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).raceWith(new WaitCommand(6)),
                new WaitCommand(1.0),
                new PrintCommand("Starting"),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).raceWith(new WaitCommand(6)),
                new PrintCommand("Starting"),
                new WaitCommand(1.0),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).raceWith(new WaitCommand(3)),
                new PrintCommand("Starting"),
                new WaitCommand(1.0),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).raceWith(new WaitCommand(3

                )));
    }


    public void zeroGyro() {
        m_pigeon.setYaw(0);
        if (RobotContainer.getAlliance() == Alliance.Blue) {
            m_megaTag1PoseEstimator = new SwerveDrivePoseEstimator(m_kinematics, Rotation2d.fromDegrees(0), m_modulePositions, new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
        } else {
            m_megaTag1PoseEstimator = new SwerveDrivePoseEstimator(m_kinematics, Rotation2d.fromDegrees(180), m_modulePositions, new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        }
        
    }

    @Override
    public void periodic() {
        
        m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());
        m_megaTag1PoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getPigeonYaw(), getModulePositions());

        m_field.getRobotObject().setPose(getPose());

        Logger.recordOutput("Swerve/Gyro", m_pigeon.getYaw().getValueAsDouble());
        Logger.recordOutput("Swerve/Pose", m_poseEstimator.getEstimatedPosition());
        Logger.recordOutput("Swerve/Module Positions", getModulePositions());
        Logger.recordOutput("Swerve/Module States", getModuleStates());
        Logger.recordOutput("Swerve/Chassis Speeds", m_chassisSpeeds);
        Logger.recordOutput("Swerve/Robot Relative Chassis Speeds", getRobotRelativeSpeeds());
        Logger.recordOutput("Swerve/Pose Estimator", m_poseEstimator.getEstimatedPosition());
    }

    public enum DriveMode {
        TELEOP,
        XWHEELS
    }
}
