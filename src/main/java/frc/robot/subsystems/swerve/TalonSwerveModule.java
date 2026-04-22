// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VelocityVoltage;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.DeviceConfigurator;
import frc.robot.util.SwerveModuleConstants;
import frc.robot.util.Utils;

public class TalonSwerveModule extends SubsystemBase implements SwerveModule {
    SwerveModuleConstants m_constants;

    private SparkMax m_steerMotor;
    private TalonFX m_driveMotor;
    private DCMotorSim m_driveMotorModel;
    private CANcoder m_steerAbsEncoder;

    private RelativeEncoder m_steerEncoder;

    private SparkClosedLoopController m_steerController;

    private SwerveModulePosition m_modulePosition;
    private SwerveModuleState m_moduleState;

    private double m_lastSpeed;
    private double m_lastAngle;

    private Rotation2d m_simAngle = new Rotation2d();

    public TalonSwerveModule(SwerveModuleConstants constants) {
        m_constants = constants;

        m_steerMotor = new SparkMax(constants.steerId, MotorType.kBrushless);
        m_driveMotor = new TalonFX(constants.driveId, new CANBus(Constants.hasCANivore() ? "*" : "rio"));
        m_driveMotorModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.04, DriveConstants.getDriveGearRatio()), DCMotor.getKrakenX60(1));
        m_steerEncoder = m_steerMotor.getEncoder();

        m_steerController = m_steerMotor.getClosedLoopController();

        m_steerAbsEncoder = new CANcoder(constants.steerEncoderId, new CANBus(Constants.hasCANivore() ? "*" : "rio"));

        m_modulePosition = new SwerveModulePosition();
        m_moduleState = new SwerveModuleState();

        DeviceConfigurator.configureSparkMaxSteerMotor(m_steerMotor);
        DeviceConfigurator.configureTalonFXDriveMotor(m_driveMotor, m_constants.driveInverted);
        DeviceConfigurator.configureCANcoder(m_steerAbsEncoder, m_constants.offset);
    }

    public double getDriveMeters() {
        return m_driveMotor.getPosition().getValue().in(Rotations);
    }

    public double getDriveMetersPerSecond() {
        return m_driveMotor.getVelocity().getValue().in(RotationsPerSecond);
    }

    public void resetAngleToAbsolute() {
        m_steerEncoder.setPosition(getAbsolutePosition());
    }

    public Rotation2d getHeading() {
        return m_moduleState.angle;
    }

    public double getAbsolutePosition() {
        return m_steerAbsEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    public SwerveModulePosition getModulePosition() {
        if (RobotBase.isReal()) {
            m_modulePosition.angle = Rotation2d.fromDegrees(m_steerEncoder.getPosition());
        } else {
            m_modulePosition.angle = m_simAngle;
        }

        m_modulePosition.distanceMeters = getDriveMeters();

        return m_modulePosition;
    }

    public SwerveModuleState getModuleState() {
        if (RobotBase.isReal()) {
            m_moduleState.angle = Rotation2d.fromDegrees(m_steerEncoder.getPosition());
        } else {
            m_moduleState.angle = m_simAngle;
        }

        m_moduleState.speedMetersPerSecond = getDriveMetersPerSecond();

        return m_moduleState;
    }

    public void runVolts(Voltage volts, double position) {
        m_steerController.setSetpoint(position, ControlType.kPosition);
        m_driveMotor.setVoltage(volts.in(Volts));
    }

    public double getDriveVoltage() {
        return m_driveMotor.getMotorVoltage().getValueAsDouble();
    }

    public void setSwerveModuleState(SwerveModuleState moduleState, boolean isOpenLoop) {
        moduleState = Utils.optimize(moduleState, getModuleHeading());

        double preCosinSpeed = moduleState.speedMetersPerSecond;
        moduleState.speedMetersPerSecond *= moduleState.angle.minus(getHeading()).getCos();

        Logger.recordOutput("Auto/Module" + m_constants.driveId + "/DesiredAngle", moduleState.angle.getDegrees());
        Logger.recordOutput("Auto/Module" + m_constants.driveId + "/CurrentAngle", getHeading().getDegrees());
        Logger.recordOutput("Auto/Module" + m_constants.driveId + "/PreCosineSpeed", preCosinSpeed);
        Logger.recordOutput("Auto/Module" + m_constants.driveId + "/PostCosineSpeed", moduleState.speedMetersPerSecond);
        Logger.recordOutput("Auto/Module" + m_constants.driveId + "/IsOpenLoop", isOpenLoop);
        Logger.recordOutput("Auto/Module" + m_constants.driveId + "/LastSpeed", m_lastSpeed);
        Logger.recordOutput("Auto/Module" + m_constants.driveId + "/SpeedCommandSent", moduleState.speedMetersPerSecond != m_lastSpeed);

        if (moduleState.angle.getDegrees() != m_lastAngle) {
            // Re-sync relative encoder to absolute before commanding new position
            // This prevents drift between the SparkMax encoder and CANcoder
            m_steerEncoder.setPosition(getAbsolutePosition());
            m_steerController.setSetpoint(moduleState.angle.getDegrees(), ControlType.kPosition);
            m_lastAngle = moduleState.angle.getDegrees();
        }

        if (moduleState.speedMetersPerSecond != m_lastSpeed) {
            if (isOpenLoop) {
                m_driveMotor.set(moduleState.speedMetersPerSecond / DriveConstants.getMaxModuleSpeed());
            } else {
                m_driveMotor.setControl(new VelocityVoltage(moduleState.speedMetersPerSecond));
            }

            m_lastSpeed = moduleState.speedMetersPerSecond;
        }

        Logger.recordOutput("Auto/Module" + m_constants.driveId + "/ActualVelocity", getDriveMetersPerSecond());
        Logger.recordOutput("Auto/Module" + m_constants.driveId + "/MotorVoltage", m_driveMotor.getMotorVoltage().getValueAsDouble());

        if (RobotBase.isSimulation()) {
            m_simAngle = moduleState.angle;
        }
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState driveMotorSim = m_driveMotor.getSimState();
        driveMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        Voltage motorVoltage = driveMotorSim.getMotorVoltageMeasure();
        m_driveMotorModel.setInputVoltage(motorVoltage.in(Volts));
        m_driveMotorModel.update(0.02);
        driveMotorSim.setRawRotorPosition(m_driveMotorModel.getAngularPosition().in(Rotations) * DriveConstants.getDriveGearRatio());
        driveMotorSim.setRotorVelocity(m_driveMotorModel.getAngularVelocity().in(RotationsPerSecond) * DriveConstants.getDriveGearRatio());
    }
}
