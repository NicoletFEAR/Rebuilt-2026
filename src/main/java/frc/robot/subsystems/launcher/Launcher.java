package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.architecture.SubsystemInterfaces.VoltageSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.DeviceIds;

public class Launcher extends SubsystemBase implements VoltageSubsystem{
    private double m_desiredVelocity;
    private double m_speedModifier = 1.0;
    private LauncherState m_state = LauncherState.OFF;
    private TalonFX m_leftMotor;
    private TalonFX m_rightMotor;
    private DCMotorSim m_leftMotorSim;
    private DCMotorSim m_rightMotorSim;

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
        leftConfig.MotionMagic.MotionMagicAcceleration = 1;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_leftMotor.getConfigurator().apply(leftConfig);

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.Feedback.SensorToMechanismRatio = LauncherConstants.getGearRatio();
        rightConfig.Slot0.kP = LauncherConstants.getKP();
        rightConfig.Slot0.kI = LauncherConstants.getKI();
        rightConfig.Slot0.kD = LauncherConstants.getKD();
        rightConfig.MotionMagic.MotionMagicAcceleration = 1;
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_rightMotor.getConfigurator().apply(rightConfig);
    }

    @Override
    public double getVoltage() {
        return Math.max(m_leftMotor.getMotorVoltage().getValueAsDouble(), m_rightMotor.getMotorVoltage().getValueAsDouble());
    }

    public void setVelocity(double velocity) {
        m_desiredVelocity = velocity;
        m_leftMotor.set(m_desiredVelocity);
        m_rightMotor.set(m_desiredVelocity);
    }

    @Override
    public void setVoltage(double voltage) {}

    public boolean isAtVelocity() {
        return Math.abs(m_leftMotor.getVelocity().getValueAsDouble() - m_desiredVelocity) < LauncherConstants.getVelocityTolerance()
            && Math.abs(m_rightMotor.getVelocity().getValueAsDouble() - m_desiredVelocity) < LauncherConstants.getVelocityTolerance();
    }

    public Command runProfileToVelocity(double velocity) {
        return new FunctionalCommand(
            () -> {
                m_desiredVelocity = velocity;
                m_leftMotor.setControl(new MotionMagicVelocityVoltage(velocity));
            },
            () -> {},
            (isFinished) -> {},
            this::isAtVelocity,
            this
        );
    }

    public Command launch() {
        return new InstantCommand(() -> m_state = LauncherState.LAUNCHING).andThen(runProfileToVelocity(LauncherConstants.getLaunchVelocity()));
    }

    public Command off() {
        return new InstantCommand(() -> m_state = LauncherState.OFF).andThen(runProfileToVelocity(LauncherConstants.getOffVelocity()));
    }

    public Command raiseSpeed() {
        return new ConditionalCommand(
            new InstantCommand(() -> m_speedModifier = Math.min(m_speedModifier + 0.1, 1.0)), 
            new InstantCommand(() -> m_speedModifier = Math.min(m_speedModifier + 0.1, 1.0)).andThen(launch()),
            () -> m_state == LauncherState.OFF
        );
    }

    public Command lowerSpeed() {
        return new ConditionalCommand(
            new InstantCommand(() -> m_speedModifier = Math.max(m_speedModifier - 0.1, 0.1)), 
            new InstantCommand(() -> m_speedModifier = Math.max(m_speedModifier - 0.1, 0.1)).andThen(launch()),
            () -> m_state == LauncherState.OFF
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Launcher Speed", 100 * m_speedModifier + "%");
        Logger.recordOutput("Launcher/Launcher/Voltage", getVoltage());
        Logger.recordOutput("Launcher/Launcher/Desired Voltage", m_desiredVelocity);
        Logger.recordOutput("Launcher/Left Launcher/Current", m_leftMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Launcher/Right Launcher/Current", m_rightMotor.getStatorCurrent().getValueAsDouble());
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
        OFF,
    }
}
