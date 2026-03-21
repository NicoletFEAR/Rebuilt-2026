package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.architecture.SubsystemInterfaces.VoltageSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.DeviceIds;

public class Launcher extends SubsystemBase implements VoltageSubsystem{
    private double m_desiredVoltage;
    private double m_desiredVelocity;
    private double m_speedModifier = 1.0;
    private LauncherState m_state = LauncherState.IDLE;
    private TalonFX m_leftMotor;
    private TalonFX m_rightMotor;
    private DCMotorSim m_leftMotorSim;
    private DCMotorSim m_rightMotorSim;

    private SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Launcher/SysIdState", state)
        ),
        new SysIdRoutine.Mechanism(
            (voltage) -> setVoltage(voltage.in(Volts)), 
            null,
            this
        )
    );

    private MotionMagicVelocityVoltage m_velocityRequest = new MotionMagicVelocityVoltage(0.0).withSlot(0);

    public Launcher() {
        m_leftMotor = new TalonFX(DeviceIds.getLeftLauncherID(), new CANBus(Constants.hasCANivore() ? "*" : "rio"));
        m_rightMotor = new TalonFX(DeviceIds.getRightLauncherID(), new CANBus(Constants.hasCANivore() ? "*" : "rio"));
        m_leftMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, LauncherConstants.getGearRatio()), DCMotor.getKrakenX60(1));
        m_rightMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, LauncherConstants.getGearRatio()), DCMotor.getKrakenX60(1));

        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.Feedback.SensorToMechanismRatio = LauncherConstants.getGearRatio();
        leftConfig.Slot0.kP = LauncherConstants.getKP();
        leftConfig.Slot0.kI = LauncherConstants.getKI();
        leftConfig.Slot0.kD = LauncherConstants.getKD();
        leftConfig.MotionMagic.MotionMagicAcceleration = 100.0;
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_leftMotor.getConfigurator().apply(leftConfig);

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.Feedback.SensorToMechanismRatio = LauncherConstants.getGearRatio();
        rightConfig.Slot0.kP = LauncherConstants.getKP();
        rightConfig.Slot0.kI = LauncherConstants.getKI();
        rightConfig.Slot0.kD = LauncherConstants.getKD();
        rightConfig.MotionMagic.MotionMagicAcceleration = 100.0;
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_rightMotor.getConfigurator().apply(rightConfig);
    }

    @Override
    public double getVoltage() {
        return Math.max(m_leftMotor.getMotorVoltage().getValueAsDouble(), m_rightMotor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void setVoltage(double voltage) {
        m_desiredVoltage = voltage;
        m_leftMotor.setControl(new VoltageOut(m_desiredVoltage));
        m_rightMotor.setControl(new VoltageOut(m_desiredVoltage));
    }

    public void setVelocity(double velocity) {
        m_desiredVelocity = velocity;
        m_leftMotor.setControl(m_velocityRequest.withVelocity(m_desiredVelocity));
        m_rightMotor.setControl(m_velocityRequest.withVelocity(m_desiredVelocity));
    }

    public boolean isAtVelocity() {
        return MathUtil.isNear(getVelocity(), LauncherConstants.getLaunchVelocity() * m_speedModifier, LauncherConstants.getVelocityTolerance());
    }

    public double getVelocity() {
        return Math.max(m_leftMotor.getVelocity().getValueAsDouble(), m_rightMotor.getVelocity().getValueAsDouble());
    }

    public Command launch() {
        return new InstantCommand(() -> {
            m_state = LauncherState.LAUNCHING;
            setVelocity(LauncherConstants.getLaunchVelocity() * m_speedModifier);
        });
    }

    public Command idle() {
        return new InstantCommand(() -> {
            m_state = LauncherState.IDLE;
            setVelocity(LauncherConstants.getIdleVelocity());
        });
    }

    public Command setSpeedModifier(double newSpeedModifier) {
        return new InstantCommand(() -> m_speedModifier = newSpeedModifier);
    }

    // TODO: Replace 0.05 with 0.1 once tuning the launcher speeds is complete
    public Command raiseSpeed() {
        return new ConditionalCommand(
            new InstantCommand(() -> m_speedModifier = Math.min(m_speedModifier + 0.05, 1.0)), 
            new InstantCommand(() -> m_speedModifier = Math.min(m_speedModifier + 0.05, 1.0)).andThen(launch()),
            () -> m_state == LauncherState.IDLE
        );
    }

    public Command lowerSpeed() {
        return new ConditionalCommand(
            new InstantCommand(() -> m_speedModifier = Math.max(m_speedModifier - 0.05, LauncherConstants.getIdleVelocity() / LauncherConstants.getLaunchVelocity())), 
            new InstantCommand(() -> m_speedModifier = Math.max(m_speedModifier - 0.05, LauncherConstants.getIdleVelocity() / LauncherConstants.getLaunchVelocity())).andThen(launch()),
            () -> m_state == LauncherState.IDLE
        );
    }

    public Command adjustSpeedToHubDistance(DoubleSupplier distance) {
        return new ConditionalCommand(
            new RunCommand(() -> m_speedModifier = MathUtil.clamp(LauncherConstants.kAutoAimSpeeds.get(distance.getAsDouble()), 0.0, 1.0)),
            new RunCommand(() -> m_speedModifier = MathUtil.clamp(LauncherConstants.kAutoAimSpeeds.get(distance.getAsDouble()), 0.0, 1.0)).andThen(launch()),
            () -> m_state == LauncherState.IDLE
        );
    }

    public Command sysId() {
        return m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(Commands.waitTime(Milliseconds.of(500)))
            .andThen(m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(Commands.waitTime(Milliseconds.of(500)))
            .andThen(m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(Commands.waitTime(Milliseconds.of(500)))
            .andThen(m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Launcher Speed", String.format("%.2f%%", m_speedModifier * 100));
        SmartDashboard.putBoolean("Launching?", m_state == LauncherState.LAUNCHING);
        Logger.recordOutput("Launcher/Speed Modifier", m_speedModifier);
        Logger.recordOutput("Launcher/Desired Voltage", m_desiredVoltage);
        Logger.recordOutput("Launcher/Desired Velocity", m_desiredVelocity);
        Logger.recordOutput("Launcher/Left/Voltage", m_leftMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Launcher/Left/Current", m_leftMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Launcher/Left/Velocity", m_leftMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Launcher/Right/Voltage", m_rightMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Launcher/Right/Current", m_rightMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Launcher/Right/Velocity", m_rightMotor.getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState leftMotorSim = m_leftMotor.getSimState();
        TalonFXSimState rightMotorSim = m_rightMotor.getSimState();
        leftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        Voltage leftMotorVoltage = leftMotorSim.getMotorVoltageMeasure();
        Voltage rightMotorVoltage = rightMotorSim.getMotorVoltageMeasure();
        m_leftMotorSim.setInputVoltage(leftMotorVoltage.in(Volts));
        m_rightMotorSim.setInputVoltage(rightMotorVoltage.in(Volts));
        m_leftMotorSim.update(0.02);
        m_rightMotorSim.update(0.02);
        leftMotorSim.setRawRotorPosition(m_leftMotorSim.getAngularPosition().in(Rotations) * LauncherConstants.getGearRatio());
        leftMotorSim.setRotorVelocity(m_leftMotorSim.getAngularVelocity().in(RotationsPerSecond) * LauncherConstants.getGearRatio());
        rightMotorSim.setRawRotorPosition(m_rightMotorSim.getAngularPosition().in(Rotations) * LauncherConstants.getGearRatio());
        rightMotorSim.setRotorVelocity(m_rightMotorSim.getAngularVelocity().in(RotationsPerSecond) * LauncherConstants.getGearRatio());
    }

    public enum LauncherState {
        LAUNCHING,
        IDLE,
    }
}
