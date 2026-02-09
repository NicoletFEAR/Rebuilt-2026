package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.architecture.SubsystemInterfaces.VoltageSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.DeviceIds;

public class Launcher extends SubsystemBase implements VoltageSubsystem{
    private double m_desiredVoltage;
    private double m_speedModifier;
    private TalonFX m_launcherLeft;
    private TalonFX m_launcherRight;
    private DCMotorSim m_motorSim;

    public Launcher() {
        m_launcherLeft = new TalonFX(DeviceIds.kLauncherLeft, Constants .hasCANivore() ? "*" : "rio");
        m_launcherRight = new TalonFX(DeviceIds.kLauncherRight, Constants.hasCANivore() ? "*" : "rio");
        m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, LauncherConstants.kGearRatio), DCMotor.getKrakenX60(1));

        m_launcherRight.setControl(new Follower(DeviceIds.kLauncherLeft, MotorAlignmentValue.Opposed));

        TalonFXConfiguration leadConfig = new TalonFXConfiguration();
        leadConfig.Feedback.SensorToMechanismRatio = LauncherConstants.kGearRatio;
        leadConfig.Slot0.kP = LauncherConstants.kP;
        leadConfig.Slot0.kI = LauncherConstants.kI;
        leadConfig.Slot0.kD = LauncherConstants.kD;
        m_launcherLeft.getConfigurator().apply(leadConfig);
        m_speedModifier = 0.1;
    }

    @Override
    public double getVoltage() {
        return m_launcherLeft.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        m_desiredVoltage = voltage;
        m_launcherLeft.setVoltage(m_desiredVoltage);
    }

    public Command launch() {
        return new InstantCommand(() -> setVoltage(LauncherConstants.kLaunchVoltage * m_speedModifier), this);
    }

    public Command off() {
        return new InstantCommand(() -> setVoltage(LauncherConstants.kOffVoltage), this);
    }

    public Command cycleSpeed() {
        return new InstantCommand(() -> {
            m_speedModifier += 0.1;
            m_speedModifier %= 1.1;
        });
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Launcher/Voltage", getVoltage());
        Logger.recordOutput("Launcher/Desired Voltage", m_desiredVoltage);
        Logger.recordOutput("Launcher/Current", m_launcherLeft.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motorSim = m_launcherLeft.getSimState();
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        Voltage motorVoltage = motorSim.getMotorVoltageMeasure();
        m_motorSim.setInputVoltage(motorVoltage.in(Volts));
        m_motorSim.update(0.02);
        motorSim.setRawRotorPosition(m_motorSim.getAngularPosition().in(Rotations) * LauncherConstants.kGearRatio);
        motorSim.setRotorVelocity(m_motorSim.getAngularVelocity().in(RotationsPerSecond) * LauncherConstants.kGearRatio);
    }
}
