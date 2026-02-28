package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.architecture.SubsystemInterfaces.VoltageSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.DeviceIds;

public class Launcher extends SubsystemBase implements VoltageSubsystem{
    private double m_desiredVoltage;
    private double m_speedModifier = 1.0;
    private LauncherState m_state = LauncherState.OFF;
    private TalonFX m_leftLauncher;
    private TalonFX m_rightLauncher;
    private DCMotorSim m_motorSim;

    public Launcher() {
        m_leftLauncher = new TalonFX(DeviceIds.getLeftLauncherID(), new CANBus(Constants.hasCANivore() ? "*" : "rio"));
        m_rightLauncher = new TalonFX(DeviceIds.getRightLauncherID(), new CANBus(Constants.hasCANivore() ? "*" : "rio"));
        m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, LauncherConstants.getGearRatio()), DCMotor.getKrakenX60(1));

        m_rightLauncher.setControl(new Follower(DeviceIds.getLeftLauncherID(), MotorAlignmentValue.Opposed));

        TalonFXConfiguration leadConfig = new TalonFXConfiguration();
        leadConfig.Feedback.SensorToMechanismRatio = LauncherConstants.getGearRatio();
        leadConfig.Slot0.kP = LauncherConstants.getKP();
        leadConfig.Slot0.kI = LauncherConstants.getKI();
        leadConfig.Slot0.kD = LauncherConstants.getKD();
        m_leftLauncher.getConfigurator().apply(leadConfig);
    }

    @Override
    public double getVoltage() {
        return m_leftLauncher.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        m_desiredVoltage = voltage;
        m_leftLauncher.setVoltage(m_desiredVoltage);
    }

    public Command launch() {
        return new InstantCommand(() -> {
            m_state = LauncherState.LAUNCHING;
            setVoltage(LauncherConstants.getLaunchVoltage() * m_speedModifier);
        }, this);
    }

    public Command off() {
        return new InstantCommand(() -> {
            m_state = LauncherState.OFF;
            setVoltage(LauncherConstants.getOffVoltage());
        }, this);
    }

    public Command raiseSpeed() {
        if (m_state == LauncherState.OFF) {
            return new InstantCommand(() -> {
                m_speedModifier = Math.min(m_speedModifier + 0.1, 1.0);
            });
        } else {
            return new InstantCommand(() -> {
                m_speedModifier = Math.min(m_speedModifier + 0.1, 1.0);
            }).andThen(launch());
        }
    }

    public Command lowerSpeed() {
        if (m_state == LauncherState.OFF) {
            return new InstantCommand(() -> {
                m_speedModifier = Math.max(m_speedModifier - 0.1, 0.1);
            });
        } else {
            return new InstantCommand(() -> {
                m_speedModifier = Math.max(m_speedModifier - 0.1, 0.1);
            }).andThen(launch());
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Launcher Speed", 100 * m_speedModifier + "%");
        Logger.recordOutput("Launcher/Launcher/Voltage", getVoltage());
        Logger.recordOutput("Launcher/Launcher/Desired Voltage", m_desiredVoltage);
        Logger.recordOutput("Launcher/Launcher/Current", m_leftLauncher.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSim = m_leftLauncher.getSimState();
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        Voltage motorVoltage = motorSim.getMotorVoltageMeasure();
        m_motorSim.setInputVoltage(motorVoltage.in(Volts));
        m_motorSim.update(0.02);
        motorSim.setRawRotorPosition(m_motorSim.getAngularPosition().in(Rotations) * LauncherConstants.getGearRatio());
        motorSim.setRotorVelocity(m_motorSim.getAngularVelocity().in(RotationsPerSecond) * LauncherConstants.getGearRatio());
    }

    public enum LauncherState {
        LAUNCHING,
        OFF,
    }
}
