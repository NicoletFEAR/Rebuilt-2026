package frc.robot.subsystems.launcher;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.architecture.SubsystemInterfaces.PositionSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.DeviceConfigurator;

public class Hood extends SubsystemBase implements PositionSubsystem {
    private SparkMax m_motor;
    private CANcoder m_encoder;
    private PIDController m_pidController;
    private SwerveDrive m_driveBase;

    private double m_desiredPosition;
    private double m_minPosition;
    private double m_maxPosition;

    private boolean m_autoTargeting;

    public Hood(SwerveDrive driveBase) {
        m_motor = new SparkMax(DeviceIds.getHoodID(), MotorType.kBrushed);
        m_encoder = new CANcoder(DeviceIds.getHoodEncoderID(), new CANBus(Constants.hasCANivore() ? "*" : "rio"));
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .p(LauncherConstants.getHoodKP())
            .i(LauncherConstants.getHoodKI())
            .d(LauncherConstants.getHoodKD());
        config.encoder.positionConversionFactor(LauncherConstants.getHoodGearRatio());
        config.smartCurrentLimit(40);
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_pidController = new PIDController(
            LauncherConstants.getHoodKP(),
            LauncherConstants.getHoodKI(),
            LauncherConstants.getHoodKD()
        );

        DeviceConfigurator.configureCANcoder(m_encoder, LauncherConstants.getHoodOffset());

        m_desiredPosition = LauncherConstants.getHoodMinPosition();
        m_minPosition = LauncherConstants.getHoodMinPosition();
        m_maxPosition = LauncherConstants.getHoodMaxPosition();

        m_motor.getEncoder().setPosition(getPosition());
        m_driveBase = driveBase;

        SmartDashboard.putData("MechSettings/Launcher/Hood/Reset Min Hood Position", new InstantCommand(() -> m_minPosition = getPosition()));
        SmartDashboard.putData("MechSettings/Launcher/Hood/Reset Max Hood Position", new InstantCommand(() -> m_maxPosition = getPosition()));
    }

    public double getPosition() {
        if (Robot.isSimulation()) {
            return m_desiredPosition;
        }

        return m_encoder.getAbsolutePosition().getValueAsDouble();
    }

    public void runToPosition(double position) {
        m_desiredPosition = position;
        m_motor.set(m_pidController.calculate(getPosition(), position));
    }

    @Override
    public Command runProfileToPosition(double position) {
        return new FunctionalCommand(
            () -> {
                m_desiredPosition = position;
                m_motor.set(m_pidController.calculate(getPosition(), m_desiredPosition));
            },
            () -> {
                m_motor.set(m_pidController.calculate(getPosition(), m_desiredPosition));
            },
            (isFinished) -> {
                m_motor.set(0.0);
            },
            this::getIsAtSetpoint,
            this
        );
    }

    public boolean getIsAtSetpoint() {
        return Math.abs(getPosition() - m_desiredPosition) < LauncherConstants.getHoodSetpointTolerance();
    }

    public Command endAutoTarget() {
        return Commands.runOnce(() -> m_autoTargeting = false).andThen(runProfileToPosition(0.0));
    }

    public Command adjustToHubDistance() {
        return new InstantCommand(() -> m_autoTargeting = true);
    }

    public void manualControl(Supplier<Double> throttle, boolean limitOverrideMode) {
        double adjustedThrottle = MathUtil.applyDeadband(throttle.get(), OperatorConstants.getOperatorControllerDeadband())
            * LauncherConstants.getHoodManualModifier();

        double newDesiredPosition = m_desiredPosition + adjustedThrottle;

         if (!limitOverrideMode) {
            newDesiredPosition = MathUtil.clamp(newDesiredPosition, m_minPosition, m_maxPosition);
        }

        m_desiredPosition = newDesiredPosition;

        if (adjustedThrottle == 0.0 && getIsAtSetpoint()) {
            m_motor.set(0.0);
        } else {
            runToPosition(m_desiredPosition);
        }
    }

    @Override
    public void periodic() {
        if (m_autoTargeting) {
            runToPosition(MathUtil.clamp(LauncherConstants.kAutoAimHoodPositions.get(m_driveBase.distanceToHub()), m_minPosition, m_maxPosition));
        }

        Logger.recordOutput("Launcher/Hood/Desired Position", m_desiredPosition);
        Logger.recordOutput("Launcher/Hood/Position", getPosition());
        Logger.recordOutput("Launcher/Hood/Current", m_motor.getOutputCurrent());
        Logger.recordOutput("Launcher/Hood/Voltage", m_motor.getBusVoltage());
        Logger.recordOutput("Launcher/Hood/Velocity", m_encoder.getVelocity().getValueAsDouble());
        Logger.recordOutput("Launcher/Hood/Is At Setpoint", getIsAtSetpoint());
    }
}
